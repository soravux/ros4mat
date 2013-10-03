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
function [ RetImages RetImgTimeStamp ] = Demo2()
% This is the code for our first demo
% Advance by step of 5cm to reach distance, at each step take IMUData 2 sec pause
% and log images and gyros data
% param[in] Distance : Distance to do in mm 
% const Step : Doing it in step of 50 mm as specified by client
Images = [];
CamTimeStamp = [];
ImageCount = 1;
AccX = [];
AccY = [];
AccZ = [];
GyroX = []; 
GyroY = []; 
GyroZ = []; 
MagX = [];
MagY = [];
MagZ = [];
IMUTimeStamp = []; 
IntervalStop = 1;
% Connect to logico agent
logico('connect', '127.0.0.1',1);
logico('subscribe', 'camera', 15, '160x120', 40, 10);
logico('subscribe', 'imu', 50, 0, 400, 5);
pause(5);

% Log cam data
[Imgs ImgTimeStamp]= logico('camera');
[w,h,c,Nbr] = size(Imgs);
NewNbr = Nbr + ImageCount - 1;
if((Nbr>0))
	CamTimeStamp = [CamTimeStamp ImgTimeStamp];
	Images(:,:,:,ImageCount:NewNbr) = Imgs(:,:,:,1:Nbr);
end
ImageCount = NewNbr+1;
% Log imu data
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
   
pause(5);

% Log cam data
[Imgs ImgTimeStamp]= logico('camera');
[w,h,c,Nbr] = size(Imgs);
NewNbr = Nbr + ImageCount - 1;
if((Nbr>0))
	CamTimeStamp = [CamTimeStamp ImgTimeStamp];
	Images(:,:,:,ImageCount:NewNbr) = Imgs(:,:,:,1:Nbr);
end
ImageCount = NewNbr+1;
% Log imu data
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

pause(5);
% Log cam data
[Imgs ImgTimeStamp]= logico('camera');
[w,h,c,Nbr] = size(Imgs);
NewNbr = Nbr + ImageCount - 1;
if((Nbr>0))
	CamTimeStamp = [CamTimeStamp ImgTimeStamp];
	Images(:,:,:,ImageCount:NewNbr) = Imgs(:,:,:,1:Nbr);
end
ImageCount = NewNbr+1;
% Log imu data
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


% Close the connection
logico('unsubscribe', 'imu');
logico('unsubscribe', 'camera');
pause(0.1);
logico('close');

% Scale the timestamps to IMUData value making sens
[zzzs,n] = size(IMUTimeStamp);
if(n>0)
    RemoveTime = IMUTimeStamp(1);
    IMUTimeStamp = IMUTimeStamp-RemoveTime;
    CamTimeStamp = CamTimeStamp-RemoveTime;
end

% Show the data
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
plot(IMUTimeStamp, GyroX);
grid on;
title('Gyroscope en X en function du temps');
xlabel('Temps');
ylabel('Valeur du Gyroscope en X');
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

% return arg image array
RetImages = Images;
RetImgTimeStamp = CamTimeStamp;