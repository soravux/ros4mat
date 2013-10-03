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
function [] = Demo4(Iteration)
% This is the code for our fourth demo
% Using batterie utilities
% param[in] Iteration : How many up and down on cpu do we want in duration of AnIterationDuration
PCBusy = 0;
Bat = [];
PC = [];
BatTimeStamp = []; 
PCTimeStamp = []; 
AnIterationDuration = 8.5  %Up or Down duration in second
% Connect to logico agent 
logico('connect', '127.0.0.1');
logico('subscribe', 'batterie', 1, 0, 50, 1);
pause(0.1);
logico('subscribe', 'ordinateur', 1, 0, 50, 1);
pause(0.1);

for i=1:Iteration*2
    if(PCBusy == 0)
        system('/home/logico/mercurial-dev/Software/Demos/runLinpack.bash')
        PCBusy = 1;
    else
        pause(AnIterationDuration);
        PCBusy = 0;
    end
    %read sampled data
	[BatData BatTS] = logico('batterie');
	[m,n] = size(BatData);
	if(m>0)
		BatTimeStamp = [BatTimeStamp BatTS];
		Bat = [Bat BatData(9, :)];
	end
	[PCData PCTS] = logico('ordinateur');
	[m,n] = size(PCData);
	if(m>0)
		PCTimeStamp = [PCTimeStamp PCTS];
		PC = [PC sum(PCData(6:8, :))]; %mean value on 1 minute change this
	end
end

% Close the connection
logico('unsubscribe', 'batterie');
pause(0.1);
logico('unsubscribe', 'ordinateur');
pause(0.1);
logico('close');

% Scale the timestamps to a value making sens
[zzzs,n] = size(BatTimeStamp);
if(n>0)
    RemoveTime = BatTimeStamp(1);
    BatTimeStamp = BatTimeStamp-RemoveTime;
    PCTimeStamp = PCTimeStamp-RemoveTime;
end
% Show Data
figure;
subplot(2,1,1);
plot(BatTimeStamp, Bat);
title('Consomation Batterie en function du temps');
xlabel('Temps');
ylabel('Consomation de la batterie');
grid on;
subplot(2,1,2);
plot(PCTimeStamp, PC);
title('Consomation PC en function du temps');
xlabel('Temps');
ylabel('Consomation du PC');
grid on;