
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

%% script to load all the parameters of interest

% number of agents
Parameter.NumberOfAgents = 8;

% parameters to specify the grid of points 
%(this is the fixed and more dense grid used for the Voronoi regions)
Parameter.GridStart = 0;      
Parameter.GridEnd = 1;
Parameter.GridStep = 0.02;

Parameter.FinerGridStep = 0.01;

% valuse of the Regularization Parameter
Parameter.RegularizationConstant = 1;

Parameter.KernelStd = 5*2*Parameter.GridStep;

Parameter.KernelType = 'gaussian';

% number of iterations
Parameter.Niter = 100;

% cost with respect to centroids or positions
Parameter.CostFlag = 'centroid';

% measurement noise standard deviation
Parameter.NoiseStd = Parameter.RegularizationConstant;

%% Parameters for the control input        

% Variance Exponential. If it is >1 we prefer exploitation if it is 
% 0<x<1 we prefer exploration. 
Parameter.VarianceExponentialShaping = 2;

% This parameter here MUST be always equal to Positions, change it just
% after the partition initialization
Parameter.VoronoiMode = 'Positions';

%% creation of the grid structure for the voronoi evaluation
% values of the x axis
Parameter.Xaxis = (Parameter.GridStart:Parameter.GridStep:Parameter.GridEnd)';

% values of the y axis
Parameter.Yaxis = (Parameter.GridStart:Parameter.GridStep:Parameter.GridEnd)';

% number of pionts in the grid
Parameter.NumberOfPoints = length(Parameter.Xaxis);

% creation of the matrix of point to manage the mesh plot
[Parameter.Xgrid,Parameter.Ygrid] = meshgrid(Parameter.Xaxis,Parameter.Yaxis);

% area of a single unitary square in the grid
Parameter.UnitArea = Parameter.GridStep^2;

%% creation of the true function evaluated in the grid and of the measurements
for h = 1:length(Parameter.Xaxis)
    for k = 1:length(Parameter.Yaxis)
        Parameter.TrueFunction(k,h) = computeSensoryFunction(Parameter.Xaxis(h),Parameter.Yaxis(k),Parameter);
    end
end

%% Initial positions
Parameter.Positions = 0.5*rand(Parameter.NumberOfAgents,2);

%% motion constraints
%Parameter.MotionConstraintsThreshold = Inf*Parameter.GridEnd;

