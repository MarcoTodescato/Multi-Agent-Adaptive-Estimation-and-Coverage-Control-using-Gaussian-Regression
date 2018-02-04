
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

function [EstimationStructure] = computeCoverage(EstimationStructure,Parameter,Variance)
% function to compute the coverage ones the density function is given. It
% computes the Voronoi regions wrt the current node positions. It computes
% the momenta relatively the regions and update the node positions trying
% to, at the same time, move towards the centroid of the region and
% minimize the ERROR in the estimation.
%
% EstimationStructure : structure that contains all the info about the
%                       particular estimation problem
% Parameter : all the parameter
% Variance: the a posteriori variance of the estimation
    
% compute Voronoi Regions
EstimationStructure.Node = computeVoronoiRegion(EstimationStructure.Node,Parameter);

% compute mass, first momentum and centroid of the regions
EstimationStructure.Node = computeMomenta(EstimationStructure.Node,Parameter,EstimationStructure.DensityFunction);

% compute additional input for uniform exploration
MaxLocations = computeMaxLocation(EstimationStructure.Node,Parameter,Variance);

% updating the position
EstimationStructure = computeUpdate(EstimationStructure,Parameter,MaxLocations,Variance);

end

