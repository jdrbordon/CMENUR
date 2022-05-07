% Copyright 2016 Jacob David Rodríguez Bordón (jacobdavid.rodriguezbordon@ulpgc.es)

% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License.

% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.

% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.

function Kep=calcular_Kep(L,A,I,E)
  Kep=zeros(6,6);
  Kep([1 4],[1 4])=E*A/L*[ 1 -1
                          -1  1];
  Kep([2 3 5 6],[2 3 5 6])=2*E*I/L*[ 6/L^2  3/L -6/L^2  3/L
                                     3/L    2   -3/L    1
                                    -6/L^2 -3/L  6/L^2 -3/L
                                     3/L    1   -3/L    2  ];
end 
