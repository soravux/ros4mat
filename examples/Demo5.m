%
%   This file is part of ros4mat.
%
%   ros4mat is free software: you can redistribute it and/or modify
%   it under the terms of the GNU Lesser General Public License as
%   published by the Free Software Foundation, either version 3 of
%   the License, or (at your option) any later version.
%
%   ros4mat is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
%   GNU Lesser General Public License for more details.
%
%    You should have received a copy of the GNU Lesser General Public
%    License along with ros4mat. If not, see <http://www.gnu.org/licenses/>.
%
function [RetImgs RetImgsTimeStamp] = Demo5(TestDurationInSecond)
% This is the code for our fourth demo
% Using batterie utilities
% param[in] TestDurationInSecond
Images = zeros(120,160,3,600);
ImageCount = 1;
GPSLat = [];
GPSLong = [];
GPSAlti = [];
GPSSpeed = [];
GPSSpeedAngle = [];
GPSALtiSpeed = [];
AccX = [];
AccY = [];
AccZ = [];
GyroX = []; 
GyroY = []; 
GyroZ = []; 
MagX = [];
MagY = [];
MagZ = [];
CamTimeStamp = []; 
IMUTimeStamp = []; 
GPSTimeStamp = []; 
DataGatheringDelay = 1; % Delay in second
% Connect to logico agent
logico('connect', '127.0.0.1',1);
logico('subscribe', 'camera', 5, '160x120', 1, 2);
logico('subscribe', 'imu', 1, 0, 1, 5);
logico('subscribe', 'gps', 1, 0, 1, 5);
pause(5);

for i=1:TestDurationInSecond
    % Log cam data
    [Imgs ImgTimeStamp]= logico('camera');
    [w,h,c,Nbr] = size(Imgs);
    if((Nbr>0) && ImageCount < 600 )
        CamTimeStamp = [CamTimeStamp ImgTimeStamp];
        Images(:,:,:,ImageCount) = Imgs(:,:,:,1);
    end
    ImageCount = ImageCount + 1;
    
    disp('Image OK');
    pause(1);
    % Get imu data
    [IMUData IMUTS] = logico('imu');
    [m,zzz] = size(IMUData);
    if(m>0)
        IMUTimeStamp = [IMUTimeStamp IMUTS];
        AccX = [AccX IMUData(1, :)];
        AccY = [AccY IMUData(2, :)];
        AccZ = [AccZ IMUData(3, :)];
        GyroX = [GyroX IMUData(4, :)];
        GyroY = [GyroY IMUData(5, :)];
        GyroZ = [GyroZ IMUData(6, :)];
        MagX = [MagX IMUData(7, :)];
        MagY = [MagY IMUData(8, :)];
        MagZ = [MagZ IMUData(9, :)];
    end
    
    disp('IMU OK');
    % Get GPS

    [GPSData GPSTS] = logico('gps');  
    [zzz,n] = size(GPSData);
    if(n>0)
        for j=1:n
            if(GPSData(j,1) ~= 0)  
                GPSTimeStamp = [GPSTimeStamp GPSTS];
                GPSLat = [GPSLat GPSData(2, :)];
                GPSLong = [GPSLong GPSData(3, :)];
                GPSAlti = [GPSAlti GPSData(4, :)];
                GPSSpeed = [GPSSpeed GPSData(5, :)];
                GPSSpeedAngle = [GPSSpeedAngle GPSData(6, :)];
                GPSALtiSpeed = [GPSALtiSpeed GPSData(7, :)];
            end
        end
    end
    pause(DataGatheringDelay);
end

% Close the connection
logico('unsubscribe', 'imu');
logico('unsubscribe', 'camera');
logico('unsubscribe', 'gps');
pause(0.1);
logico('close');

% Scale the timestamps to a value making sens
[zzzs,n] = size(IMUTimeStamp);
if(n>0)
    RemoveTime = IMUTimeStamp(1);
    IMUTimeStamp = IMUTimeStamp-RemoveTime;
    CamTimeStamp = CamTimeStamp-RemoveTime;
    %GPSTimeStamp = GPSTimeStamp-RemoveTime;
end

% Show DataIMU
figure;
plot(IMUTimeStamp, AccX);
grid on;
title('accel en X en function du temps');
xlabel('Temps');
ylabel('Valeur de l''accel en X');
figure;
plot(IMUTimeStamp, AccY);
grid on;
title('accel en Y en function du temps');
xlabel('Temps');
ylabel('Valeur de l''accel en Y');
figure;
plot(IMUTimeStamp, AccZ);
grid on;
title('accel en Z en function du temps');
xlabel('Temps');
ylabel('Valeur de l''accel en Z');
figure;
grid on;
plot(IMUTimeStamp, GyroX);
title('Gyroscope en X en function du temps');
xlabel('Temps');
ylabel('Valeur du Gyroscope en X');
figure;
grid on;
plot(IMUTimeStamp, GyroY);
title('Gyroscope en Y en function du temps');
xlabel('Temps');
ylabel('Valeur Gyroscope en Y');
figure;
plot(IMUTimeStamp, GyroZ);
grid on;
title('Gyroscope en Z en function du temps');
xlabel('Temps');
ylabel('Valeur Gyroscope en Z');
figure;
plot(IMUTimeStamp, MagX);
grid on;
title('Magn en X en function du temps');
xlabel('Temps');
ylabel('Valeur du Magn en X');
figure;
plot(IMUTimeStamp, MagY);
grid on;
title('Magn en Y en function du temps');
xlabel('Temps');
ylabel('Valeur du Magn en Y');
figure;
plot(IMUTimeStamp, MagZ);
grid on;
title('Magn en Z en function du temps');
xlabel('Temps');
ylabel('Valeur du Magn en Z');
% GPS data
figure;
plot(GPSTimeStamp, GPSLat);
grid on;
title('GPS : Latitude en function du temps');
xlabel('Temps');
ylabel('Valeur de la latitude');
figure;
plot(GPSTimeStamp, GPSLong);
grid on;
title('GPS: Longitude en function du temps');
xlabel('Temps');
ylabel('Valeur de la Longitude');
figure;
plot(GPSTimeStamp, GPSAlti);
grid on;
title('Altitude en function du temps');
xlabel('Temps');
ylabel('Valeur de l''altitude');
figure;
plot(GPSTimeStamp, GPSSpeed);
grid on;
title('Vitesse en function du temps');
xlabel('Temps');
ylabel('Valeur de la vitesse');
figure;
plot(GPSTimeStamp, GPSSpeedAngle);
grid on;
title('Angle de la vitesse en (x,y) en function du temps');
xlabel('Temps');
ylabel('Valeur de la vitesse en (x,y)');
figure;
plot(GPSTimeStamp, GPSALtiSpeed);
grid on;
title('Vitesse selon l''axe de l''altitude en function du temps');
xlabel('Temps');
ylabel('Valeur de vitesse d''altitude');


%Return images 
RetImgs = Images;
RetImgsTimeStamp = CamTimeStamp;
