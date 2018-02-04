
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

function Minv = generalSchurInverse(Ainv,B,D)
%SCHURINVERSE computes the inverse using the Schur decomposition having the
%second block being a scalar.

lenA = length(Ainv);
lenD = length(D);
Minv = zeros(lenA+lenD);

AinvB = mtimes(Ainv,B);
M22notInv = D - B'*AinvB;
M22B = M22notInv\B';

Minv(end-lenD+1:end,end-lenD+1:end) = inv(M22notInv);
tmp = mtimes(AinvB,mtimes(M22B,Ainv));
Minv(1:lenA,1:lenA) = Ainv + tmp;
Minv(1:lenA,end-lenD+1:end) = -AinvB/M22notInv;
Minv(end-lenD+1:end,1:lenA) = -M22B*Ainv;


end

