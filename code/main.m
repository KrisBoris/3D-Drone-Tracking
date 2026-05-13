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

% Pobieranie parametrów kamery
camera_calibration = readtable("cameras_calibration.csv", "Delimiter", ";");
num_of_cameras = length(camera_calibration.cam_name);

% Wyświelt dla sprawdzenia
% disp(camera_calibration);

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