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
function [SS] = TakeASnapShot(Format, Wide, Height)
VidFormat = strcat(Format, '_', int2str(Wide), 'x', int2str(Height));
vid = videoinput('winvideo', 1 ,VidFormat);
SS = getsnapshot(vid);
%delete(vid);
