
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

function  EstimationStructure = computeUpdate(EstimationStructure,Parameter,MaxLocations,Variance)
% compute the new position of the nodes
% INPUT: Node : structure of the nodes
%        Parameter
%        GradienteDirection : direction computer by computeError
%        Variance: Variance of the estimation
%
% OUTPUT : Node : structure updated

Node = EstimationStructure.Node;

% computes the centroids for all the agents
Centroid = zeros(Parameter.NumberOfAgents,2);
for j=1:Parameter.NumberOfAgents
    Centroid(j,:) = Node(j).C;
end                     
                         
for i = 1:Parameter.NumberOfAgents
    
    MaxVariance = Variance(MaxLocations(i,2),MaxLocations(i,1));
    State = rand > MaxVariance^(Parameter.VarianceExponentialShaping);
       
    % update of the positon of the node i
    if( State == 0)
        
        % updating the number of agents exploring
        EstimationStructure.NumberOfAgentsExploring(Parameter.CurrentIteration) = ...
            EstimationStructure.NumberOfAgentsExploring(Parameter.CurrentIteration) + 1;
                
        % max location 
        pos = [Parameter.Xgrid(1,MaxLocations(i,1)),...
              Parameter.Ygrid(MaxLocations(i,2),1)];
       
        % saturator
        pos = min([Parameter.GridEnd Parameter.GridEnd],...
                   max([Parameter.GridStart Parameter.GridStart],pos));
    else
                
        pos = Centroid(i,:);

    end
    
    % saturate motion if limited capabilities
%     if norm(Node(i).pos - pos) > Parameter.MotionConstraintsThreshold
%         
%         slope = atan2((pos(2) - Node(i).pos(2)),(pos(1) - Node(i).pos(1)));
%         pos_sat = Node(i).pos + Parameter.MotionConstraintsThreshold * ...
%                                     [real(exp(1i*slope)) imag(exp(1i*slope))];
%         pos = pos_sat;
%     end
    
    EstimationStructure.averageEnergy(Parameter.CurrentIteration) = ...
        EstimationStructure.averageEnergy(Parameter.CurrentIteration) + norm(pos-Node(i).pos)/Parameter.NumberOfAgents;
    
    Node(i).pos = pos;
    
end

% update neighbors positions
newCurrentPositions = node2pos(Node);
for i=1:Parameter.NumberOfAgents

    Node(i).pvicini = newCurrentPositions(Node(i).vicini,:);
end

% Checking if two or more nodes are in the same position
for i=1:Parameter.NumberOfAgents
    
    % flag isChangedThePosition OFF
    isChangeThePosition = 0;
    
    while(min(sum(abs([Node(i).pos(1)*ones(Node(i).Ni,1) Node(i).pos(2)*ones(Node(i).Ni,1)] - Node(i).pvicini),2)) <= 0.05)
        
        % flag isChangedThePosition ON
        isChangeThePosition = 1;
        
        %update to a random position close to the current position
        pos = Node(i).pos + (rand(1,2)-0.5)*0.2;
        
        % checking the boundary and fitting the position into the grid
        Node(i).pos = min([Parameter.GridEnd Parameter.GridEnd],...
                      max([Parameter.GridStart Parameter.GridStart],pos));
    end
    
    if( isChangeThePosition )
        %fprintf('Overlapping-->');
        for j = Node(i).vicini
            Node(j).pvicini(Node(j).vicini == i,:) = pos;
        end
        %fprintf('Solved');
    end
end

EstimationStructure.Node = Node;


end

