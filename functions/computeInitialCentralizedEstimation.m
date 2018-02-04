
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

function RecursiveEstimation = computeInitialCentralizedEstimation(RecursiveEstimation,Parameter)
% This function computes the initialization for the centralized estimation 
% and for the posterior variance
% INPUT: RecursriveEstimation, structure with all the information about the
%                              estimation
%        Parameter, Structure with all the simulation parameters 
% OUTPUT: RecursriveEstimation, structure with all the updated information 
%                               about the estimation 


NumberOfLocations = size(RecursiveEstimation.InputLocations,1);

RecursiveEstimation.SampledKernel = computeKernel(RecursiveEstimation.InputLocations(:,1),RecursiveEstimation.InputLocations(:,2), Parameter.KernelStd);

Autocovariance = eye(NumberOfLocations)/(RecursiveEstimation.SampledKernel +...
                 Parameter.RegularizationConstant*eye(NumberOfLocations));
             
RecursiveEstimation.InvertedSampledKernel = Autocovariance;
             
%% RecursiveEstimation.SampledKernel = Autocovariance;

EstimationCoefficients = Autocovariance*RecursiveEstimation.InputLocations(:,3);
          
RecursiveEstimation.DensityFunction = zeros(Parameter.NumberOfPoints);
                     
for i=1:Parameter.NumberOfPoints
    for j=1:Parameter.NumberOfPoints
       KernelRow = exp( -((Parameter.Xaxis(i)*ones(1,NumberOfLocations) - ...
                           RecursiveEstimation.InputLocations(1:NumberOfLocations,1)').^2 + ...
                          (Parameter.Yaxis(j)*ones(1,NumberOfLocations) - ...
                           RecursiveEstimation.InputLocations(1:NumberOfLocations,2)').^2 ) ...
                          / (Parameter.KernelStd^2));
       RecursiveEstimation.DensityFunction(j,i) = RecursiveEstimation.DensityFunction(j,i) + KernelRow*EstimationCoefficients;
       RecursiveEstimation.APosterioriVariance(j,i) = 1 - KernelRow*Autocovariance*KernelRow';
    end
end
             

end