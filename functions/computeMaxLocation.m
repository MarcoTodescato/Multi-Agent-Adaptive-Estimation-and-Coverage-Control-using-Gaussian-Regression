
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

function MaxLocations = computeMaxLocation(Node,Parameter,Variance)
% for each voronoi region compute the location of the max of the posterior
% inside the region
%
% INPUT : Node
%         Parameter
%         Variance, the a posteiori variance
%
% OUTPUT : MaxDirection

MaxLocations = zeros(Parameter.NumberOfAgents,2);

% extract the gradient for each node
for i=1:Parameter.NumberOfAgents;
    subFunction = Variance.*Node(i).VoronoiRegion;
    %[~,ind] = max(subFunction(:));
    m = max(subFunction(:));
    ind = find(subFunction(:) == m);
    n = length(ind);
    ind = ind(randperm(n,1));
    [nodeMaxX,nodeMaxY] = ind2sub(size(Variance),ind);
    MaxLocations(i,:) = [nodeMaxY,nodeMaxX];
end


end

