
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

function [EstimationStructure, Parameter] = transmitNewMeasurements( Parameter, EstimationStructure )
% This function simulate data transmission from 
% the nodes to the Central base station.

    % getting the positions of the nodes
    PositionRecursiveEstimation = node2pos(EstimationStructure.Node);
  
    % new measurements
    noisyMeasurements = zeros(Parameter.NumberOfAgents,1);
    for i = 1:Parameter.NumberOfAgents
        noisyMeasurements(i) = computeSensoryFunction(PositionRecursiveEstimation(i,1),...
                                    PositionRecursiveEstimation(i,2), ...
                                    Parameter)...
                               + Parameter.NoiseStd...
                                 *randn(1);
    end                    
    % take new measurements in the new input locations
    EstimationStructure.InputLocations = ...
                                    [EstimationStructure.InputLocations;...
                                     PositionRecursiveEstimation,...
                                     noisyMeasurements];
end

