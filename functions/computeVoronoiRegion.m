
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

function Node = computeVoronoiRegion(Node,Parameter)
% function to compute the Voronoi regions

% if the centroids are not available yet, e.g., at the first iteration,
% compute the Voronoi regions respect to the robot positions
if strcmp(Parameter.VoronoiMode , 'Positions')

    for i=1:Parameter.NumberOfAgents
        Node(i).VoronoiRegion = ones(Parameter.NumberOfPoints);
        for j=1:Node(i).Ni
            
            if Node(i).pos(2)==Node(i).pvicini(j,2)
                Intercept = (Node(i).pos(1) + Node(i).pvicini(j,1))/2;
                if Node(i).pos(1) - Intercept >=0
                    tempVoronoi = (Parameter.Xgrid(:) - Intercept) >= zeros(Parameter.NumberOfPoints^2,1);
                else
                    tempVoronoi = (Parameter.Xgrid(:) - Intercept) <= zeros(Parameter.NumberOfPoints^2,1);
                end
            else
                Slope = (Node(i).pos(1)-Node(i).pvicini(j,1))/(Node(i).pvicini(j,2)-Node(i).pos(2));
                Intercept = (Node(i).pos(2)+Node(i).pvicini(j,2))/2 - Slope*(Node(i).pos(1)+Node(i).pvicini(j,1))/2;
                if Node(i).pos(2) - Slope*Node(i).pos(1) - Intercept >= 0
                    tempVoronoi = (Parameter.Ygrid(:) - Slope*Parameter.Xgrid(:) - Intercept) >= zeros(Parameter.NumberOfPoints^2,1);
                else
                    tempVoronoi = (Parameter.Ygrid(:) - Slope*Parameter.Xgrid(:) - Intercept) <= zeros(Parameter.NumberOfPoints^2,1);
                end
            end
            
            Node(i).VoronoiRegion = bitand(Node(i).VoronoiRegion,vec2mat(tempVoronoi,Parameter.NumberOfPoints));
        end
        Node(i).VoronoiRegion = Node(i).VoronoiRegion';
    end

else 
    
    Centroids = zeros(Parameter.NumberOfAgents,2);
    for i = 1:Parameter.NumberOfAgents
        Centroids(i,:) = Node(i).C;   
    end

    for i=1:Parameter.NumberOfAgents
        Node(i).VoronoiRegion = ones(Parameter.NumberOfPoints);
        for j=1:Node(i).Ni
            
            if Centroids(i,2)==Centroids(Node(i).vicini(j),2)
                Intercept = (Centroids(i,1) + Centroids(Node(i).vicini(j),1))/2;
                if Centroids(i,1) - Intercept >=0
                    tempVoronoi = (Parameter.Xgrid(:) - Intercept) >= zeros(Parameter.NumberOfPoints^2,1);
                else
                    tempVoronoi = (Parameter.Xgrid(:) - Intercept) <= zeros(Parameter.NumberOfPoints^2,1);
                end
            else
                Slope = (Centroids(i,1)-Centroids(Node(i).vicini(j),1))/(Centroids(Node(i).vicini(j),2)-Centroids(i,2));
                Intercept = (Centroids(i,2)+Centroids(Node(i).vicini(j),2))/2 - Slope*(Centroids(i,1)+Centroids(Node(i).vicini(j),1))/2;
                if Centroids(i,2) - Slope*Centroids(i,1) - Intercept >= 0
                    tempVoronoi = (Parameter.Ygrid(:) - Slope*Parameter.Xgrid(:) - Intercept) >= zeros(Parameter.NumberOfPoints^2,1);
                else
                    tempVoronoi = (Parameter.Ygrid(:) - Slope*Parameter.Xgrid(:) - Intercept) <= zeros(Parameter.NumberOfPoints^2,1);
                end
            end
            Node(i).VoronoiRegion = bitand(Node(i).VoronoiRegion,reshape(tempVoronoi,Parameter.NumberOfPoints,Parameter.NumberOfPoints)');
        end
        Node(i).VoronoiRegion = Node(i).VoronoiRegion';
    end

end
    
end

