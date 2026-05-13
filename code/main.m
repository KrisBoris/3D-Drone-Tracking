% 3D drone tracking project
% Skład sekcji:
% Krzysztof Borys
% Jakub Poroszewski
% Agnieszka Zięcina

clc, clearvars, close all, format compact

% 1. Pobierz parametry kamer
% 2. Wyznacz macierze projekcji
% 3. Pobierz współrzędne [x,y] z obrazów
% 4. Wyznacz punkty [x,y,z]
% 5. Prównaj z rzeczywistymi wartościami
% 6. Jeżeli działa -> pochwal się że sam to napisałeś
%    Jeżeli nie działa -> to chat to tak napisał

% Dodać korekcję dystorsji (dane są w pliku kalibracji kamery)
% Dodać obsługę błędów (np. brak danych)
% Dodać porównywanie z rzeczywistymi wartościami
% Naniesć współrzędne na film z lotu drona (opcjonalnie, jak będzie czas i
% chęci..., czyli tej funkcji najpewnej nie będzie...)

% Dla dwóch dronów albo osobny program, albo zmodyfikować ten - dopytać



% Pobieranie parametrów kamery
camera_calibration = readtable("cameras_calibration.csv", "Delimiter", ";");
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


% Układ kolumn w pliku .csv
% 1 - nr klatki obrazu
% 2 - środek obiektu na osi x
% 3 - środek obiektu na osi y
% 4 - szerokość ramki
% 5 - wysokość ramki
% 6 - współrzędna x prawego górnego rogu
% 7 - współrzędna y prawego górnego rogu
% 8 - pewność należności obiektu do przypisanej mu klasy (confidence)

% Ścieżki do plików
file_paths = {
    "dl_data/55260362.csv", ...
    "dl_data/60881257.csv", ...
    "dl_data/61016180.csv", ...
    "dl_data/61920488.csv"
};

% Dane z YOLOv5 dla każdej z kamer
drone_data = cell(4, 1);
for i = 1:num_of_cameras
    drone_data{i} = readmatrix(file_paths{i});
end

% Współrzędne x, y drona na obrazie
drone_coords = cell(4, 1);
for i = 1:num_of_cameras
    drone_coords{i} = drone_data{i}(:, [2,3]);
end

% Wyznaczanie współrzędnych x,y,z
frames = length(drone_data{1}(:, 1));
trajectory_3d = NaN(frames, 3);

for i = 1:frames
    
    L = zeros(num_of_cameras*2, 3);
    B = zeros(num_of_cameras*2, 1);

    for cam = 1:num_of_cameras

       point = drone_coords{cam}(i, :);

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
    xyz = pinv(L) * B;
    trajectory_3d(i, :) = xyz';
end

% Wyświetlanie trajektori lotu drona
plot3(trajectory_3d(:, 1), trajectory_3d(:, 2), trajectory_3d(:, 3));