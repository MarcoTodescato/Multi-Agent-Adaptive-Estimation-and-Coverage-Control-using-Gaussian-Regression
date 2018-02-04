
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

function [node] = nodeInit(Positions,Parameter)
% creazione della struttura con le informazioni di ogni nodo
% Positions: position of the nodes in the grid
% Parameter: parameter of the grid

node = struct('vicini',{}, 'Ni',{},'pvicini',{},'pos',{},...
              'VoronoiRegion',{},'M',{},'L',{},'C',{});

for i=1:Parameter.NumberOfAgents
    vicini = 1:Parameter.NumberOfAgents;
    vicini(find(vicini==i)) = [];
    node(i).vicini = vicini;
    node(i).Ni = length(node(i).vicini);
    node(i).pvicini = Positions(vicini,:);
    node(i).pos = Positions(i,:);
    node(i).VoronoiRegion = ones(Parameter.NumberOfPoints);
end
clear vicini

end

