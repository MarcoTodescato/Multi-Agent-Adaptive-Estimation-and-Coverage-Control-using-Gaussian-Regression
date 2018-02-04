
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

clear
close all
clc

% add current path
addpath(genpath('./'))

% load necessary parameters
loadParameter;

% load estimation structure
loadEstimationStructure;

% standard coverage (without estimation - perfect knowledge of the sensory function)
Parameter.NoEstimation = computeClassicCoverage(Parameter);

%% recursive estimation
tStartToT = tic;   % to save to total iteration time
IterationTime = zeros(Parameter.Niter,1);  % to save all the partial times

% evaluation of the cost function
RecursiveEstimation.CostFunction(1) = computeCost(Parameter,RecursiveEstimation.Node,Parameter.TrueFunction,Parameter.CostFlag);

% init the max of the posterior on the finer grid
maxPosterior = ones(Parameter.Niter,1);

Parameter.CurrentIteration = 2;
while Parameter.CurrentIteration <= Parameter.Niter
    
    tStart = tic;
               
    % transmit measurement to central server
    [RecursiveEstimation, Parameter] = transmitNewMeasurements( Parameter, RecursiveEstimation );
    
    % compute the function estimate
    RecursiveEstimation = computeCentralizedEstimation(RecursiveEstimation,Parameter);
    
    % compute max posterior on finer grid 
    %(uncomment these lines for comparison with fixed grid)
    %tComputeMaxStart = tic;
    %maxPosteriorFiner(Parameter.CurrentIteration) = computeMaxPosteriorFinerGrid(Parameter,RecursiveEstimation);
    %tComputeMaxStop = toc(tComputeMaxStart);
    
    % perform the coverage
    RecursiveEstimation = computeCoverage(RecursiveEstimation,Parameter,RecursiveEstimation.APosterioriVariance);
          
    % evaluation of the cost function
    RecursiveEstimation.CostFunction(Parameter.CurrentIteration) = ...
                        computeCost(Parameter,RecursiveEstimation.Node,Parameter.TrueFunction,Parameter.CostFlag);

    % update timing vector
    IterationTime(Parameter.CurrentIteration) = IterationTime(Parameter.CurrentIteration -1) +  toc(tStart); %- tComputeMaxStop;

    % update the pointer to current iteration
    Parameter.CurrentIteration = Parameter.CurrentIteration + 1;
    
end
tStopTot = toc(tStartToT);

%%
plotResults


