
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

function [Node] = computeMomenta(Node,Parameter,DensityFunction)
% computes the voronoi momenta
% Node : structure of node
% Parameter : all the parameter
% DensityFunction : density over the area of interest

DensityFunction(DensityFunction<0) = 1e-10;

for i=1:Parameter.NumberOfAgents
    
    % mass
    Node(i).M = Parameter.UnitArea*sum(sum(DensityFunction.*Node(i).VoronoiRegion));
    
    % 
    Node(i).L = [Parameter.UnitArea*sum(sum(Parameter.Xgrid.*DensityFunction.*Node(i).VoronoiRegion)) , ...
                 Parameter.UnitArea*sum(sum(Parameter.Ygrid.*DensityFunction.*Node(i).VoronoiRegion))];
    
    % centroid         
	Node(i).C = Node(i).L/Node(i).M;
end



end

