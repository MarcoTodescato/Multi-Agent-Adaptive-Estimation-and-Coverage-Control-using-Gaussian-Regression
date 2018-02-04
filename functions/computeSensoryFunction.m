
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

function f = computeSensoryFunction(x,y,Parameter)
% function to compute the sampled value of the function f_mu in the point
% (x1,x2)

nx = length(x);
ny = length(y);

f = zeros(ny,nx);

 for i = 1:nx
     for j = 1:ny
         
         f(j,i) = 5 * exp(-((x(i)-Parameter.GridEnd*0.8).^2 + ...
             (y(j)-Parameter.GridEnd*0.2).^2)./...
             (Parameter.GridEnd*0.1)^2) + ...
             5 * exp(-((x(i)-Parameter.GridEnd*0.5).^2 +...
             (y(j)-Parameter.GridEnd*0.7).^2)./...
             (Parameter.GridEnd*0.1)^2);
     end
 end
end


