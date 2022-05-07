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

function escribir_resultados(modelo,fichero)
%escribir_resultados Escribir los resultados

  % Desplazamientos y giro de los nodos
  f=fopen([fichero '.u.txt'],'w');
  fprintf(f,'__nodo ___________ux ___________uy ___________ur\n');
  for n=1:modelo.n_nodos
    fprintf(f,'%6d %13.6e %13.6e %13.6e\n',n,modelo.nodos_u(n,:));
  end
  fclose(f);

  % Esfuerzos en los nodos de los elementos
  f=fopen([fichero '.F.txt'],'w');
  fprintf(f,'__elem ___________Ni ___________Vi ___________Mi ___________Nj ___________Vj ___________Mj\n');
  for n=1:modelo.n_elementos
    fprintf(f,'%6d %13.6e %13.6e %13.6e %13.6e %13.6e %13.6e\n',n,modelo.elementos_esfuerzos(n,:));
  end
  fclose(f);

  % Reacciones en los apoyos
  f=fopen([fichero '.R.txt'],'w');
  fprintf(f,'_apoyo ___________Rx ___________Ry ___________Rt\n');
  for n=1:modelo.n_apoyos
    fprintf(f,'%6d %13.6e %13.6e %13.6e\n',n,modelo.apoyos_R(n,:));
  end
  fclose(f);

end
