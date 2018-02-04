
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

function K = computeKernel(X,Y,StdScale)
%create the kernel function
% the input location can be mono or bidimensional. If they are mono just
% collect all the x values in the X vector and set Y=zeros(N,1), while if
% they are bidimensional just put the x entries in the X vector and the y
% in the Y vector.

% lenght of the vector
N = length(X);

% compute the kernel matrix in the input location given
K = exp( -(((X*ones(1,N) - ones(N,1)*X').^2 + (Y*ones(1,N) - ones(N,1)*Y').^2) )/(StdScale^2));
end


