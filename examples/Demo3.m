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
function Demo3(Distance)
% This is the code for our third demo
% Advance to reach distance, and take in analog entry data
% and data
% param[in] Distance : Distance to do in mm 
FreqAcqui = 200;
BufferSize = 2000;
FreqUpdate = 10;

Step = 100;
StepToDo = ceil(Distance./Step);
ADCOne = [];
ADCTwo = [];
ADCTimeStamp = [];
AccX = []; 
AccY = []; 
AccZ = []; 
IMUTimeStamp = []; 
% Connect to logico agent
logico('connect', '127.0.0.1');
logico('subscribe', 'adc', FreqAcqui, 16+32, BufferSize, FreqUpdate)
pause(0.1);
logico('subscribe', 'imu', FreqAcqui, 0, BufferSize, FreqUpdate)
pause(0.1);

for i=1:StepToDo
    pause(3);
    [ADCData ADCTS] = logico('adc');
    % ADC Data
    [m,zzz] = size(ADCData);
    if(m>0)
        ADCTimeStamp = [ADCTimeStamp ADCTS];
        ADCOne = [ADCOne ADCData(5, :)];
        ADCTwo = [ADCTwo ADCData(6, :)];
    end
    % Get imu data
    [IMUData IMUTS] = logico('imu');
    [m,zzz] = size(IMUData);
    if(m>0)
        IMUTimeStamp = [IMUTimeStamp IMUTS];
        AccX = [AccX IMUData(1, :)];
        AccY = [AccY IMUData(2, :)];
        AccZ = [AccZ IMUData(3, :)];
    end
end
% Close logico connection
logico('unsubscribe','adc');
pause(0.1);
logico('unsubscribe', 'imu');
pause(0.1);
logico('close');

% Scale the timestamps to a value making sens
[zzzs,n] = size(IMUTimeStamp);
if(n>0)
    RemoveTime = IMUTimeStamp(1);
    IMUTimeStamp = IMUTimeStamp-RemoveTime;
    ADCTimeStamp = ADCTimeStamp-RemoveTime;
end

figure;
plot(ADCTimeStamp, ADCOne);
grid on;
title('ADC 1 en function du temps');
xlabel('Temps');
ylabel('Valeur de channel 1');
figure;
plot(ADCTimeStamp, ADCTwo);
grid on;
title('ADC 2 en function du temps');
xlabel('Temps');
ylabel('Valeur du channel 2');
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

