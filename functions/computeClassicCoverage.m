
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

function NoEstimation = computeClassicCoverage(Parameter)
%function to perform the classic coverage according to Llyod algorithm
%assuming perfect knowledge of the sensory function. The function, given a
%set of parameters as input, returns a structure containing all the
%information related to the performed coverage

NoEstimation = Parameter.NoEstimation;
centroid = zeros(Parameter.NumberOfAgents,2);

NoEstimation.CostFunction(1) = computeCost(Parameter,NoEstimation.Node,Parameter.TrueFunction,Parameter.CostFlag);

for t = 2:Parameter.Niter
    
    NoEstimation.Node = computeVoronoiRegion(NoEstimation.Node,Parameter);
    NoEstimation.Node = computeMomenta(NoEstimation.Node,Parameter,Parameter.TrueFunction);
    for i=1:Parameter.NumberOfAgents
        centroid(i,:) = NoEstimation.Node(i).C;
    end
    for i=1:Parameter.NumberOfAgents
        NoEstimation.averageEnergy(t) = NoEstimation.averageEnergy(t) +...
                      norm(centroid(i,:) -NoEstimation.Node(i).pos)...
                      /Parameter.NumberOfAgents;
        NoEstimation.Node(i).pos = centroid(i,:);
        support = centroid;
        support(i,:) = [];
        NoEstimation.Node(i).pvicini = support;         
    end
    
    % Checking if two or more nodes are in the same position
    for i=1:Parameter.NumberOfAgents
        
        % flag isChangedThePosition OFF
        isChangeThePosition = 0;
        
        while(min(sum(abs([NoEstimation.Node(i).pos(1)*ones(NoEstimation.Node(i).Ni,1)...
                NoEstimation.Node(i).pos(2)*ones(NoEstimation.Node(i).Ni,1)] - ...
                NoEstimation.Node(i).pvicini),2)) <= 0.05)
            
            % flag isChangedThePosition ON
            isChangeThePosition = 1;
            
            %update to a random position close to the current position
            pos = NoEstimation.Node(i).pos + (rand(1,2)-0.5)*0.2;
            
            % checking the boundary and fitting the position into the grid
            NoEstimation.Node(i).pos = min([1 1],max([0 0],pos));
        end
        
        if( isChangeThePosition )
            fprintf('Overlapping-->');
            for j = NoEstimation.Node(i).vicini
                NoEstimation.Node(j).pvicini(NoEstimation.Node(j).vicini == i,:) = pos;
            end
            fprintf('Solved');
        end
    end
    NoEstimation.CostFunction(t) = ...
                   computeCost(Parameter,NoEstimation.Node,Parameter.TrueFunction,Parameter.CostFlag);
                                  
end

end

