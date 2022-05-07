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

function psi=calcular_psi(xi,L)
%calcular_psi Devuelve las funciones de forma de la viga Euler-Bernoulli
% dando como entrada la coordenada xi [0,1] y la longitud L.
% v = psi(1)*vi + psi(2)*thetai + psi(3)*vj + psi(4)*thetaj
  psi=zeros(4,1);
  psi(1)=1-3*xi^2+2*xi^3;
  psi(2)=L*xi*(1-xi)^2;
  psi(3)=3*xi^2-2*xi^3;
  psi(4)=L*xi^2*(xi-1);
end
