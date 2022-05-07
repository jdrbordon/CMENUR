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

function [K,f]=ensamblar(modelo)
%ensamblar Ensamblar la matriz de rigidez y vector de cargas de la
%estructura

  % ----------------------------------------------- %
  % Calcular matriz de rigidez de la estructura (K) %
  % ----------------------------------------------- %

  % Predimensionar K
  K=zeros(3*modelo.n_nodos,3*modelo.n_nodos);
  % Ensamblar matrices de rigidez elementales
  for ke=1:modelo.n_elementos;
    %
    % Datos basicos del elemento
    %
    ni=modelo.elementos_nodos(ke,1);   % Nodo i
    nj=modelo.elementos_nodos(ke,2);   % Nodo j
    xi=modelo.nodos_x(ni,:);           % Vector de posicion nodo i
    xj=modelo.nodos_x(nj,:);           % Vector de posicion nodo j
    r=xj-xi;                           % Vector distancia entre nodos
    elemento_L=sqrt(dot(r,r));         % Longitud
    alfa=atan2(r(2),r(1));             % Angulo
    s=modelo.elementos_seccion(ke);    % Seccion
    elemento_A=modelo.secciones_A(s);  % Area
    elemento_I=modelo.secciones_I(s);  % Inercia
    m=modelo.elementos_material(ke);   % Material
    elemento_E=modelo.materiales_E(m); % Modulo de Young
    %
    % Calcular matriz rigidez elemental en coordenadas globales (Ke)
    %
    Kep=calcular_Kep(elemento_L,elemento_A,elemento_I,elemento_E); % Ke'
    Le=calcular_Le(alfa);                                          % Le
    Ke=Le*Kep*Le';                                                 % Ke
    %
    % Ensamblar Ke en K
    %
    GDLE=[3*ni-2 3*ni-1 3*ni 3*nj-2 3*nj-1 3*nj]; % Indice de los GDL del elemento
    K(GDLE,GDLE)=K(GDLE,GDLE)+Ke;                 % Sumar Ke en K
  end

  
  % ------------------------------------- %
  % Calcular vector de cargas nodales (f) %
  % ------------------------------------- %

  % Predimensionar f
  f=zeros(3*modelo.n_nodos,1);
  
  % Ensamblar vectores de cargas nodales
  for kc=1:modelo.n_cargas
    n=modelo.cargas_nodo(kc); % Nodo al que esta conectada la carga
    F=modelo.cargas_F(kc,:)'; % Fuerza nodal
    GDLN=[3*n-2 3*n-1 3*n];   % Indice de los GDL del nodo
    f(GDLN,1)=f(GDLN,1)+F;    % Sumar F en f
  end
  
  % Ensamblar vectores de cargas transversales
  for kc=1:modelo.n_cargasq
    ke=modelo.cargasq_elemento(kc);    % Elemento al que esta conectada la carga
    %
    % Datos basicos del elemento
    %
    ni=modelo.elementos_nodos(ke,1);   % Nodo i
    nj=modelo.elementos_nodos(ke,2);   % Nodo j
    xi=modelo.nodos_x(ni,:);           % Vector de posicion nodo i
    xj=modelo.nodos_x(nj,:);           % Vector de posicion nodo j
    r=xj-xi;                           % Vector distancia entre nodos
    elemento_L=sqrt(dot(r,r));         % Longitud
    alfa=atan2(r(2),r(1));             % Angulo
    Le=calcular_Le(alfa);              % u=Le·u'
    q=modelo.cargasq_q(kc,1);          % Valor de la carga transversal
    %
    % Vector de cargas equivalentes en coordenadas locales (opuesto a las
    % cargas de empotramiento)
    %
    Q=zeros(6,1);
    Q(2)= q*elemento_L/2;
    Q(3)= q*elemento_L^2/12;
    Q(5)= q*elemento_L/2;
    Q(6)=-q*elemento_L^2/12;
    %
    % Ensamblar
    %
    GDLE=[3*ni-2 3*ni-1 3*ni 3*nj-2 3*nj-1 3*nj]; % Indice de los GDL del elemento
    f(GDLE)=f(GDLE)+Le*Q;                         % Sumar Le*Q en f
  end 
  

  % -------------------------------------------------- %
  % Modificar K y f para apoyos inclinados y elasticos %
  % -------------------------------------------------- %
  
  % Recorrer los apoyos y operar en aquellos que lo necesitan
  for ka=1:modelo.n_apoyos
    switch modelo.apoyos_tipo(ka)
      %
      % Apoyo inclinado: girar los grados de libertad globales del nodo
      % hasta grados de libertad donde ux' es el desplazamiento sobre
      % el plano inclinado y uy' es el desplazamiento perpendicular (nulo)
      %
      case {4,8}
        n=modelo.apoyos_nodo(ka);    % Nodo al que esta conectado el apoyo
        alfa=modelo.apoyos_alfa(ka); % Angulo del plano inclinado
        Ln=[cos(alfa) -sin(alfa)     % Matriz de rotacion u=L·u'
            sin(alfa)  cos(alfa)];
        GDLN=[3*n-2 3*n-1];          % Indice de los GDL ux, uy del nodo
        K(:,GDLN)=K(:,GDLN)*Ln;      % Girar las variables
        K(GDLN,:)=Ln'*K(GDLN,:);     % Girar las ecuaciones en K
        f(GDLN)=Ln'*f(GDLN);         % Girar las ecuaciones en f
      %
      % Apoyo elastico: ensamblar matriz de rigidez del apoyo
      %
      case 9
        n=modelo.apoyos_nodo(ka);     % Nodo al que esta conectado el apoyo
        Kx=modelo.apoyos_K(ka,1);     % Rigidez en direccion x
        Ky=modelo.apoyos_K(ka,2);     % Rigidez en direccion y
        Kr=modelo.apoyos_K(ka,3);     % Rigidez de rotacion
        Ka=[Kx,0,0;0,Ky,0;0,0,Kr];    % Matriz de rigidez del apoyo
        GDLN=[3*n-2 3*n-1 3*n];       % Indice de los GDL del nodo
        K(GDLN,GDLN)=K(GDLN,GDLN)+Ka; % Sumar Ke en K
    end
  end

end

