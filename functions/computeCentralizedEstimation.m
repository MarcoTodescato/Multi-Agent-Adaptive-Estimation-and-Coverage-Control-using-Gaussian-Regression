
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

function RecursiveEstimation = computeCentralizedEstimation(RecursiveEstimation,Parameter) 
% This function computes the centralized estimation of a function 
% and its the posterior variance
% INPUT: RecursiveEstimation, structure with all the information about the
%                              estimation
%        Parameter, Structure with all the simulation parameters 
% OUTPUT: RecursriveEstimation, structure with all the updated information 
%                               about the estimation 
                                     
% special update for the first iteration
if Parameter.CurrentIteration == 2
    RecursiveEstimation = computeInitialCentralizedEstimation(RecursiveEstimation,Parameter);
    RecursiveEstimation.maxAPosterioriVariance(Parameter.CurrentIteration) = max(max(RecursiveEstimation.APosterioriVariance));
    RecursiveEstimation.minAPosterioriVariance(Parameter.CurrentIteration) = min(min(RecursiveEstimation.APosterioriVariance));
    RecursiveEstimation.averageAPosterioriVariance(Parameter.CurrentIteration) = sum(sum(RecursiveEstimation.APosterioriVariance))/Parameter.NumberOfPoints^2;
    return;
end

NumberOfLocations = size(RecursiveEstimation.InputLocations,1);
OldNumberOfLocations = NumberOfLocations - Parameter.NumberOfAgents;

NewSampledKernel = zeros(NumberOfLocations);
NewSampledKernel(1:OldNumberOfLocations,1:OldNumberOfLocations) = RecursiveEstimation.SampledKernel;
RecursiveEstimation.SampledKernel = NewSampledKernel;

for i=1:Parameter.NumberOfAgents
    SampledKernelColumn = exp( -((RecursiveEstimation.InputLocations(1:OldNumberOfLocations+i,1) -...
                                  RecursiveEstimation.InputLocations(OldNumberOfLocations+i,1)).^2 +...
                                 (RecursiveEstimation.InputLocations(1:OldNumberOfLocations+i,2) -...
                                  RecursiveEstimation.InputLocations(OldNumberOfLocations+i,2)).^2 )...
                                 / (Parameter.KernelStd^2));    
    RecursiveEstimation.SampledKernel(1:OldNumberOfLocations+i,OldNumberOfLocations+i) = SampledKernelColumn;
    RecursiveEstimation.SampledKernel(OldNumberOfLocations+i,1:OldNumberOfLocations+i-1) = SampledKernelColumn(1:end-1);
end

RecursiveEstimation.InvertedSampledKernel = generalSchurInverse(RecursiveEstimation.InvertedSampledKernel,...
                                                                RecursiveEstimation.SampledKernel(1:end-Parameter.NumberOfAgents,end-Parameter.NumberOfAgents+1:end),...
                                                                RecursiveEstimation.SampledKernel(end-Parameter.NumberOfAgents+1:end,end-Parameter.NumberOfAgents+1:end)+...
                                                                Parameter.RegularizationConstant*sparse(eye(Parameter.NumberOfAgents)));


Autocovariance = RecursiveEstimation.InvertedSampledKernel;   

% estimation of the coefficient filtering the measurements
EstimationCoefficients = Autocovariance*RecursiveEstimation.InputLocations(:,3);

Kbar = kernelEvaluation(Parameter.Xaxis,Parameter.Yaxis,RecursiveEstimation.InputLocations, Parameter.KernelStd);
RecursiveEstimation.DensityFunction = reshape(Kbar'*EstimationCoefficients,Parameter.NumberOfPoints,Parameter.NumberOfPoints);
RecursiveEstimation.APosterioriVariance = ones(Parameter.NumberOfPoints) - ...
            reshape(sum(bsxfun(@times,Kbar,Autocovariance*Kbar)),Parameter.NumberOfPoints,Parameter.NumberOfPoints);

RecursiveEstimation.maxAPosterioriVariance(Parameter.CurrentIteration) = max(max(RecursiveEstimation.APosterioriVariance));
RecursiveEstimation.minAPosterioriVariance(Parameter.CurrentIteration) = min(min(RecursiveEstimation.APosterioriVariance));
RecursiveEstimation.averageAPosterioriVariance(Parameter.CurrentIteration) = sum(sum(RecursiveEstimation.APosterioriVariance))/Parameter.NumberOfPoints^2;


end