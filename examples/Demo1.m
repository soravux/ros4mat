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
function [ ] = Demo1(Distance)
% This is the code for our first demo
% Advance by step of 5cm to reach distance, at each step take a 2 sec pause
% and log images and gyros data
% param[in] Distance : Distance to do in mm 
% const Step : Doing it in step of 50 mm as specified by client
Step = 50;
StepToDo = ceil(Distance./Step);
GyroX = []; 
GyroY = []; 
GyroZ = []; 
IMUTimeStamp = []; 
ImgCount = 1;

% Connect to logico agent
logico('connect', '127.0.0.1', 0);
logico('subscribe', 'camera', 5, '800x600', 1, 1);
pause(5);
logico('subscribe', 'imu', 50, 0, 500, 50);

for i=1:StepToDo
    [IMUData IMUTS] = logico('imu');
    %Imu Data
    [m,zzz] = size(IMUData);
    if(m>0)
        IMUTimeStamp = [IMUTimeStamp IMUTS];
        GyroX = [GyroX IMUData(4, :)];
        GyroY = [GyroY IMUData(5, :)];
        GyroZ = [GyroZ IMUData(6, :)];
    end
    % Log cam data
    [Imgs ImgTimeStamp]= logico('camera');
    [w,h,c,nbr] = size(Imgs);
    if(nbr > 0)
        figure;
        imshow(Imgs(:,:,:,1));
        ImgCount = ImgCount + 1;
    end
end
% Close the connection
logico('unsubscribe', 'imu');
logico('unsubscribe', 'camera');
pause(2);
logico('close');

% Scale the timestamps to a value making sens
[zzzs,n] = size(IMUTimeStamp);
if(n>0)
    IMUTimeStamp = IMUTimeStamp-IMUTimeStamp(1);
end

% Show data
figure;
plot(IMUTimeStamp, GyroX);
title('Gyroscope en X en function du temps');
xlabel('Temps');
ylabel('Valeur Gyroscope en X');

figure;
plot(IMUTimeStamp, GyroY);
grid on;
title('Gyroscope en Y en function du temps');
xlabel('Temps');
ylabel('Valeur Gyroscope en Y');

figure;
plot(IMUTimeStamp, GyroZ);
grid on;
title('Gyroscope en Z en function du temps');
xlabel('Temps');
ylabel('Valeur Gyroscope en Z');
