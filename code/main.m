% 3D drone tracking project
% Skład sekcji:
% Krzysztof Borys
% Jakub Poroszewski
% Agnieszka Zięcina

clc, clearvars, close all, format compact

% 1. Pobierz parametry kamer
% 2. Wyznacz macierze projekcji P
% 3. Wyznacz macierze fundamentalne F
% 4. Pobierz współrzędne [x,y] z obrazów
% 5. Wyznacz pierwszy punkt [x,y,z] + linie epipolarne
% 6. Wyznacz punkty [x,y,z] i dopasuj do drona na podstawie linii
%    epipolarnych
% 7. Wyznacz nowe linie epipoalrne dla aktualnych punktów
% 8. Powtórz 6 i 7 dla wszystkich klatek obrazu
% 9. Wyświetl wyniki
% 10. Jeżeli działa -> pochwal się że sam to napisałeś
%    Jeżeli nie działa -> to chat to tak napisał

% Pobieranie parametrów kamery
camera_calibration = readtable("two_drones/cameras_calibration.csv", "Delimiter", ";");
num_of_cameras = length(camera_calibration.cam_name);

% Ogniskowa
f = camera_calibration.focal_length(1);

% Współrzędne środka układu współrzędnych (takie same dla wszystkich kamer)
cx = camera_calibration.cx(1);
cy = camera_calibration.cy(1);

% Macierz intrinsics (parametrów wewnętrznych)
K = [f 0 cx;
     0 f cy;
     0 0 1];

% Pusta macierz projekcji
P = cell(4, 1);

% Wyznaczanie macierzy projekcji dla każdej kamery
for i = 1:num_of_cameras
    
    % Wektor translacji
    t = [camera_calibration.cam_x_mm_(i); camera_calibration.cam_y_mm_(i); camera_calibration.cam_z_mm_(i)];

    % Wartości wektora rotacji
    rx = camera_calibration.cam_or_x_rad_(1);
    ry = camera_calibration.cam_or_y_rad_(1);
    rz = camera_calibration.cam_or_z_rad_(1);
    
    % Macierz rotacji
    R = eul2rotm([rz ry rx], 'ZYX');

    % Macierz projekcji
    P{i} = K * [R t];    
end


% Macierze fundamentalne F
F = cell(num_of_cameras, num_of_cameras);

% Wyznaczanie macierzy fundamentalnych F
for i = 1:num_of_cameras
    for j = i+1:num_of_cameras

        % Essential Matrix E = [t]_x * R
        R_rel = P{j}(1:3,1:3) * P{i}(1:3,1:3)';     % Rj * Ri'
        t_rel = P{j}(1:3,4) - R_rel * P{i}(1:3,4);  % tj - Rj*Ri'*ti

        % Skew-symmetric matrix
        tx = [ 0 -t_rel(3) t_rel(2);
               t_rel(3) 0 -t_rel(1);
               -t_rel(2) t_rel(1) 0 ];

        E = tx * R_rel;

        % Wzór macierzy fundamentalnej F = K2^{-T} * E * K1^{-1}
        K1_inv = inv(K);
        K2_invT = inv(K)';
        F{i,j} = K2_invT * E * K1_inv;
        F{j,i} = F{i,j}';     
    end
end


% Układ kolumn w pliku .csv (dla drona 1 i 2)
% 1 - nr klatki obrazu
% 2. 9 - środek obiektu na osi x
% 3. 10 - środek obiektu na osi y
% 4. 11 - szerokość ramki
% 5. 12 - wysokość ramki
% 6, 13 - współrzędna x prawego górnego rogu
% 7, 14 - współrzędna y prawego górnego rogu
% 8, 15 - pewność należności obiektu do przypisanej mu klasy (confidence)

% Ścieżki do plików
file_paths = {
    "two_drones/55260362.csv", ...
    "two_drones/60881257.csv", ...
    "two_drones/61016180.csv", ...
    "two_drones/61920488.csv"
    };

% Dane z YOLOv5 dla każdej z kamer
num_drones = 2;
drone_data = cell(num_of_cameras, num_drones);
for i = 1:num_of_cameras
    for j = 1:num_drones
        drone_data{i, j} = readmatrix(file_paths{i});
    end
end

% Współrzędne x, y dronów na obrazie
drone_coords = cell(num_of_cameras, num_drones);
for i = 1:num_of_cameras
    for j = 1:num_drones
        drone_coords{i, j} = drone_data{i}(:, [2 + (7 * (j - 1)),  3 + (7 * (j - 1))]);
    end
end

frames = size(drone_data{1}, 1);

% Struktura: epipolar_lines{ref_cam}{target_cam}{frame} = [A B C]
epipolar_lines = cell(num_of_cameras, num_of_cameras);

% Trajektoria dronów
trajectory_3d = cell(2,1);
trajectory_3d{1} = NaN(frames, 3);
trajectory_3d{2} = NaN(frames, 3);

% Obliczanie współrzędnych x,y,z oraz linii epipolarnych dla pierwszej
% klatki obrazu
for drone = 1:num_drones
    L = []; B = [];
    for cam = 1:num_of_cameras
        pt = drone_coords{cam, drone}(1, :);
        if any(isnan(pt)), continue; end
        x = pt(1); y = pt(2);
        Pcam = P{cam};

        row1 = [Pcam(1,1)-x*Pcam(3,1), Pcam(1,2)-x*Pcam(3,2), Pcam(1,3)-x*Pcam(3,3)];
        row2 = [Pcam(2,1)-y*Pcam(3,1), Pcam(2,2)-y*Pcam(3,2), Pcam(2,3)-y*Pcam(3,3)];

        L = [L; row1; row2];
        B = [B; x*Pcam(3,4)-Pcam(1,4); y*Pcam(3,4)-Pcam(2,4)];
    end
    if size(L,1) >= 4
        xyz = pinv(L) * B;
        trajectory_3d{drone}(1, :) = xyz';
    end

    % Obliczanie linii epipolarnych dla drona w pierwszej klatce
    for ref = 1:num_of_cameras
        for tgt = 1:num_of_cameras
            if ref == tgt || isempty(F{ref,tgt})
                continue;
            end

            % Współrzędne x,y,z drona w pierwszej klatce obrazu
            pt = drone_coords{ref, drone}(1, :);

            if any(isnan(pt))
                epipolar_lines{drone}{ref}{tgt}{1} = NaN(1,3);
                continue;
            end

            homog_pt = [pt, 1];
            line_abc = homog_pt * F{ref, tgt};
            % Normalizacja i dodanie eps w celu uniknięcia mnożenia przez 0
            line_abc = line_abc / norm(line_abc(1:2) + eps);

            epipolar_lines{drone}{ref}{tgt}{1} = line_abc;
        end
    end
end


% Wyznaczanie współrzędnych x,y,z
for i = 2:frames

    xyz = cell(2, 1);

    for drone = 1:num_drones

        L = zeros(num_of_cameras*2, 3);
        B = zeros(num_of_cameras*2, 1);

        for cam = 1:num_of_cameras

            point = drone_coords{cam, drone}(i, :);

            % Pomiń w przypadku braku detekcji
            if any(isnan(point))
                continue
            end

            x = point(1);
            y = point(2);
            Pcam = P{cam};

            % Rekonstrukcja koordynatów 3D
            L(cam, :) = [Pcam(1,1) - x*Pcam(3,1), Pcam(1,2) - x*Pcam(3,2), Pcam(1,3) - x*Pcam(3,3)];
            B(cam) = x*Pcam(3,4) - Pcam(1,4);
            L(cam + 1, :) = [Pcam(2,1) - y*Pcam(3,1), Pcam(2,2) - y*Pcam(3,2), Pcam(2,3) - y*Pcam(3,3)];
            B(cam + 1) = y*Pcam(3,4) - Pcam(2,4);
        end

        % Obliczanie współrzędnych 3D
        xyz{drone} = pinv(L) * B;
        trajectory_3d{drone}(i, :) = xyz{drone}';
    end         

    % Dron do którego zostanie przypisany dany punkt

    % Maksmyalna odległość punktu od linii epipolarnej - punkt znajdujący
    % się dalej od niej zostaje odrzucony
    dist_threshold = 1.0e+06;   % najlepsze wyniki dla 1.0e+06 (nie, nie wiem czemu)
    
    assignment = [0, 0];

    for det = 1:num_drones
        min_dist = Inf;
        closest_drone = 0;

        for dron_prev = 1:num_drones
            total_dist = 0;
            pair_count = 0;

            for ref = 1:num_of_cameras
                for tgt = ref+1:num_of_cameras
                    if isempty(epipolar_lines{dron_prev}{ref}{tgt}{i-1})
                        continue;
                    end

                    pt_ref = drone_coords{ref, det}(i, :);
                    pt_tgt = drone_coords{tgt, det}(i, :);

                    if any(isnan(pt_ref)) || any(isnan(pt_tgt))
                        continue;
                    end

                    dist1 = pointToEpipolarDistance(pt_tgt, epipolar_lines{dron_prev}{ref}{tgt}{i-1});
                    dist2 = pointToEpipolarDistance(pt_ref, epipolar_lines{dron_prev}{tgt}{ref}{i-1});

                    if ~isnan(dist1) && ~isnan(dist2)
                        total_dist = total_dist + (dist1 + dist2)/2;
                        pair_count = pair_count + 1;
                    end
                end
            end

            if pair_count > 0
                avg_dist = total_dist / pair_count;
                if avg_dist < min_dist
                    min_dist = avg_dist;
                    closest_drone = dron_prev;
                end
            end
        end
        
        if min_dist < dist_threshold
            assignment(det) = closest_drone;
        else
            % Odrzucenie punktu
            assignment(det) = 0;
        end
    end

    for det = 1:2
        assigned_drone = assignment(det);
        if assigned_drone > 0
            trajectory_3d{assigned_drone}(i, :) = xyz{det}';
        end
        % Jeśli nie przypisano (0) - zostaje NaN
    end

    % Obliczanie linii epipolarnych dla aktualnej klatki
    for drone = 1:num_drones
        for ref = 1:num_of_cameras
            for tgt = 1:num_of_cameras
                if ref == tgt || isempty(F{ref,tgt})
                    continue;
                end
    
                pt = drone_coords{ref, drone}(i, :);
    
                if any(isnan(pt))
                    epipolar_lines{drone}{ref}{tgt}{i} = NaN(1,3);
                    continue;
                end
    
                homog_pt = [pt, 1];
                line_abc = homog_pt * F{ref, tgt};
                line_abc = line_abc / norm(line_abc(1:2) + eps);
    
                epipolar_lines{drone}{ref}{tgt}{i} = line_abc;
            end
        end
    end
end


% Wyświetlanie trajektori lotu drona
figure('Name', 'Trajektorie lotu dwóch dronów', 'NumberTitle', 'off', 'Position', [100 100 900 700]);
hold on;

% Dron 1 (niebieski)
plot3(trajectory_3d{1}(:,1), trajectory_3d{1}(:,2), trajectory_3d{1}(:,3), ...
      'b.-', 'LineWidth', 1.0, 'MarkerSize', 5, 'DisplayName', 'Dron 1');

% Dron 2 (czerwony)
plot3(trajectory_3d{2}(:,1), trajectory_3d{2}(:,2), trajectory_3d{2}(:,3), ...
      'r.-', 'LineWidth', 1.0, 'MarkerSize', 5, 'DisplayName', 'Dron 2');

grid on;
xlabel('X [mm]');
ylabel('Y [mm]');
zlabel('Z [mm]');
title('Trajektorie lotu dwóch dronów');
legend('Location', 'best');
view(3);
axis equal;
hold off;