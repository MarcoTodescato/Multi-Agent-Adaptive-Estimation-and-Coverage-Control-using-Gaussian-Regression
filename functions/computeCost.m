
% Multi-agents adaptive estimation and coverage control using Gaussian regression 
%
% Copyright (C) 2017, University of Padova
% Andrea Carron , carrona@ethz.ch
% Marco Todescato, mrc.todescato@gmail.com
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.

function H = computeCost(Parameter,Node,DensityFunction,flag)
% this function computes the value of the cost function in the current
% either with respect to the centroid of the area monitored by each agent
% or with respect their current position

if nargin < 4
    flag = 'centroid';
end

Normalizer = sum(sum(DensityFunction))*Parameter.UnitArea;
H = 0;

switch flag
        
    case 'position'
        
        for i=1:Parameter.NumberOfAgents
            H = H + sum(sum(sqrt( ...
                (Node(i).pos(1)*ones(Parameter.NumberOfPoints)-Parameter.Xgrid).^2+...
                (Node(i).pos(2)*ones(Parameter.NumberOfPoints)-Parameter.Ygrid).^2).*...
                DensityFunction.*Node(i).VoronoiRegion./Normalizer))*Parameter.UnitArea;
        end

    otherwise
        
        for i=1:Parameter.NumberOfAgents
            H = H + sum(sum(sqrt( ...
                (Node(i).C(1)*ones(Parameter.NumberOfPoints)-Parameter.Xgrid).^2+...
                (Node(i).C(2)*ones(Parameter.NumberOfPoints)-Parameter.Ygrid).^2).*...
                DensityFunction.*Node(i).VoronoiRegion./Normalizer))*Parameter.UnitArea;
        end
end

end

