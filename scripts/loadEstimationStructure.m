
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

%% script for the creation of the structures of nodes together with some 
% additional computations

Parameter.VoronoiMode = 'Positions';

% structure related to the Recursive Estimation
RecursiveEstimation.Node = nodeInit(Parameter.Positions,Parameter);

% initial density function
RecursiveEstimation.DensityFunction = ones(Parameter.NumberOfPoints);

% initial posterior
RecursiveEstimation.APosterioriVariance = ones(Parameter.NumberOfPoints);

% energy at each iteration
RecursiveEstimation.averageEnergy = zeros(Parameter.Niter,1);

% structure for just coverage and no estimation
NoEstimation = RecursiveEstimation;

% max of the posterior at each iteration
RecursiveEstimation.maxAPosterioriVariance = ones(Parameter.Niter,1);

% average of the posterior at each iteration
RecursiveEstimation.minAPosterioriVariance = ones(Parameter.Niter,1);

% min of the posterior at each iteration
RecursiveEstimation.averageAPosterioriVariance = ones(Parameter.Niter,1);

% evolution of the cost function
RecursiveEstimation.CostFunction = zeros(Parameter.Niter,1);

% all the input locations
RecursiveEstimation.InputLocations = [];

% Number of agents which are making exploration
RecursiveEstimation.NumberOfAgentsExploring = zeros(1,Parameter.Niter);
RecursiveEstimation.NumberOfAgentsExploring(1) = Parameter.NumberOfAgents;

% compute Voronoi Regions for Recursive Estimation
RecursiveEstimation.Node = computeVoronoiRegion(RecursiveEstimation.Node,Parameter);

% compute mass, first momentum and centroid of the regions for Recursive Estimation
RecursiveEstimation.Node = computeMomenta(RecursiveEstimation.Node,Parameter,RecursiveEstimation.DensityFunction);

% evolution of the cost function
NoEstimation.CostFunction = zeros(Parameter.Niter,1);

% compute Voronoi Regions for Only Coverage
NoEstimation.Node = computeVoronoiRegion(NoEstimation.Node,Parameter);

% compute mass, first momentum and centroid of the regions for Only Coverage
NoEstimation.Node = computeMomenta(NoEstimation.Node,Parameter,Parameter.TrueFunction);

% Saving the NoEstimation Structure in Parameter
Parameter.NoEstimation = NoEstimation;

% setting this variable to 'Centroid' the voronoi will be computed wrt Centroid
% using 'Positions' will be computed wrt the positions of the agents
Parameter.VoronoiMode = 'Centroid';
