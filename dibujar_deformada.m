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

function dibujar_deformada(modelo)
%dibujar_deformada Dibuja la deformada

  % Modelo 2D
  dim=2;

  % Abrir figura y configurarla para dibujar varias cosas
  figure(2);
  close(2);
  figure(2);
  hold on;

  % Coordenadas maximas y minimas, y dimensiones de la estructura
  xmin=zeros(dim,1);
  xmax=zeros(dim,1);
  L=zeros(dim,1);
  for i=1:dim
    xmin(i)=min(modelo.nodos_x(:,i));
    xmax(i)=max(modelo.nodos_x(:,i));
    L(i)=xmax(i)-xmin(i);
  end
  % Dimension caracteristica
  D=max(L);

  % Ejes de la grafica
  margen=0.25;
  rango=zeros(2*dim,1);
  for i=1:dim
    rango((2*i-1):(2*i))=[xmin(i)-margen*D xmax(i)+margen*D];
  end
  axis(rango)

  % ---------------------- %
  % ESTRUCTURA INDEFORMADA %
  % ---------------------- %

  % Dibujar los elementos en posicion indeformada
  for n=1:modelo.n_elementos
    line(modelo.nodos_x(modelo.elementos_nodos(n,1:2),1),...
      modelo.nodos_x(modelo.elementos_nodos(n,1:2),2),...
      'LineWidth',1,'Color','b')
  end
  
  % El tamano de los apoyos sera de un 10% de la dimension caracteristica
  tamano=0.1*D;
  
  % Dibujar
  for n=1:modelo.n_apoyos
    x=modelo.nodos_x(modelo.apoyos_nodo(n),:);
    switch modelo.apoyos_tipo(n)
      % Apoyo tipo fijo articulado
      case 1
        dibujo=crear_dibujo_apoyo('fijo');
        dibujar_polilineas(dibujo,tamano,modelo.apoyos_beta(n,1),x,1,'k')
      % Apoyo tipo carro horizontal articulado
      case 2
        dibujo=crear_dibujo_apoyo('carro_articulado');
        dibujar_polilineas(dibujo,tamano,modelo.apoyos_beta(n,1),x,1,'k')
      % Apoyo tipo carro vertical articulado
      case 3
        dibujo=crear_dibujo_apoyo('carro_articulado');
        dibujar_polilineas(dibujo,tamano,pi/2+modelo.apoyos_beta(n,1),x,1,'k')
      % Apoyo tipo carro inclinado articulado
      case 4
        dibujo=crear_dibujo_apoyo('carro_articulado');
        dibujar_polilineas(dibujo,tamano,modelo.apoyos_beta(n,1),x,1,'k')
      % Apoyo tipo fijo empotrado
      case 5
        dibujo=crear_dibujo_apoyo('empotramiento');
        dibujar_polilineas(dibujo,tamano,modelo.apoyos_beta(n,1),x,1,'k') 
      % Apoyo tipo carro horizontal empotrado
      case 6
        dibujo=crear_dibujo_apoyo('carro_empotrado');
        dibujar_polilineas(dibujo,tamano,modelo.apoyos_beta(n,1),x,1,'k')
      % Apoyo tipo carro vertical empotrado
      case 7
        dibujo=crear_dibujo_apoyo('carro_empotrado');
        dibujar_polilineas(dibujo,tamano,pi/2+modelo.apoyos_beta(n,1),x,1,'k')
      % Apoyo tipo carro inclinado empotrado 
      case 8
        dibujo=crear_dibujo_apoyo('carro_empotrado');
        dibujar_polilineas(dibujo,tamano,modelo.apoyos_beta(n,1),x,1,'k')
      % Apoyo elastico 
      case 9
        dibujo=crear_dibujo_apoyo('resorte_vertical');
        dibujar_polilineas(dibujo,tamano,modelo.apoyos_beta(n,1),x,1,'k') 
        dibujar_polilineas(dibujo,tamano,pi/2+modelo.apoyos_beta(n,2),x,1,'k') 
        dibujo=crear_dibujo_apoyo('resorte_giro');
        dibujar_polilineas(dibujo,tamano,pi/4+modelo.apoyos_beta(n,3),x,1,'k')
    end
  end

  % Dibujar los nodos de apoyos articulados
  for n=1:modelo.n_apoyos
    x=modelo.nodos_x(modelo.apoyos_nodo(n),:);
    switch modelo.apoyos_tipo(n)
      case {1,2,3,4}
      plot(x(1),x(2),'ok','MarkerSize',6)
    end
  end  
  
  % -------------------- %
  % ESTRUCTURA DEFORMADA %
  % -------------------- %
  
  % Maximo desplazamiento
  max_u=max(sqrt(modelo.nodos_u(:,1).^2+modelo.nodos_u(:,2).^2+(modelo.nodos_u(:,3)*D).^2));

  % Factor multiplicador de los desplazamientos
  factor_u=0.1*D/max_u;
  
  % Dibujar los elementos deformados
  np=20; % Numero de puntos que se cogen para dibujar elemento
  for ke=1:modelo.n_elementos
    % Calcular 
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
    Le=calcular_Le(alfa);              % u = Le * u'
    % Buscar si hay carga transversal
    pos=find(modelo.cargasq_elemento==ke);
    if length(pos)==1
      q=modelo.cargasq_q(pos,1);
    else
      q=0;
    end
    % Vector elemental de desplazamientos en coordenadas globales
    u=[modelo.nodos_u(ni,1)
       modelo.nodos_u(ni,2)
       modelo.nodos_u(ni,3)
       modelo.nodos_u(nj,1)
       modelo.nodos_u(nj,2)
       modelo.nodos_u(nj,3)];
    % Vector elemental de desplazamientos en coordenadas locales
    up=Le'*u;
    % Inicializar puntos del elemento (xe) y los desplazamientos (ue)
    % correspondientes
    xe=zeros(np,2);
    ue=zeros(np,2);
    uptmp=zeros(2,1);
    % Barrer puntos del elemento de acuerdo a np
    for kchi=1:np
      % Coordenada local chi
      chi=(kchi-1)/(np-1);
      %
      % Deformada debida a cargas nodales
      %
      % Funciones de forma Euler-Bernoulli (desplazamiento transversal uy')
      psi=calcular_psi(chi,elemento_L);
      % Funciones de forma desplazamiento axial ux' (y de la geometria tambien)
      phi(1)=1-chi;
      phi(2)=chi;
      % Coordenadas x de cada coordenada local chi
      xe(kchi,1)=phi(1)*xi(1)+phi(2)*xj(1);
      xe(kchi,2)=phi(1)*xi(2)+phi(2)*xj(2);
      % Desplazamiento axial
      uptmp(1)=phi(1)*up(1)+phi(2)*up(4);
      % Desplazamiento transversal
      uptmp(2)=psi(1)*up(2)+psi(2)*up(3)+psi(3)*up(5)+psi(4)*up(6);
      %
      % Deformada debida a carga transversal
      %
      uptmp(2)=uptmp(2)+q*elemento_L^4/elemento_E/elemento_I/24*chi^2*(chi^2-2*chi+1);
      % Transformar a globales
      ue(kchi,:)=Le(1:2,1:2)*uptmp;
    end
    line(xe(:,1)+factor_u*ue(:,1),xe(:,2)+factor_u*ue(:,2),'LineWidth',3,'Color','b')
  end
  
  % El tamano de los apoyos sera de un 10% de la dimension caracteristica
  tamano=0.1*D;
  
  % Dibujar
  for n=1:modelo.n_apoyos
    x=modelo.nodos_x(modelo.apoyos_nodo(n),:);
    u=modelo.nodos_u(modelo.apoyos_nodo(n),:);
    x=x+factor_u*u(1:2);
    switch modelo.apoyos_tipo(n)
      % Apoyo tipo fijo articulado
      case 1
        dibujo=crear_dibujo_apoyo('fijo');
        dibujar_polilineas(dibujo,tamano,modelo.apoyos_beta(n,1),x,2,'k')
      % Apoyo tipo carro horizontal articulado
      case 2
        dibujo=crear_dibujo_apoyo('carro_articulado');
        dibujar_polilineas(dibujo,tamano,modelo.apoyos_beta(n,1),x,2,'k')
      % Apoyo tipo carro vertical articulado
      case 3
        dibujo=crear_dibujo_apoyo('carro_articulado');
        dibujar_polilineas(dibujo,tamano,pi/2+modelo.apoyos_beta(n,1),x,2,'k')
      % Apoyo tipo carro inclinado articulado 
      case 4
        dibujo=crear_dibujo_apoyo('carro_articulado');
        dibujar_polilineas(dibujo,tamano,modelo.apoyos_beta(n,1),x,2,'k')
      % Apoyo tipo fijo empotrado
      case 5
        dibujo=crear_dibujo_apoyo('empotramiento');
        dibujar_polilineas(dibujo,tamano,modelo.apoyos_beta(n,1),x,2,'k') 
      % Apoyo tipo carro horizontal empotrado
      case 6
        dibujo=crear_dibujo_apoyo('carro_empotrado');
        dibujar_polilineas(dibujo,tamano,modelo.apoyos_beta(n,1),x,2,'k')
      % Apoyo tipo carro vertical empotrado
      case 7
        dibujo=crear_dibujo_apoyo('carro_empotrado');
        dibujar_polilineas(dibujo,tamano,pi/2+modelo.apoyos_beta(n,1),x,2,'k')
      % Apoyo tipo carro inclinado empotrado 
      case 8
        dibujo=crear_dibujo_apoyo('carro_empotrado');
        dibujar_polilineas(dibujo,tamano,modelo.apoyos_beta(n,1),x,2,'k')
      % Apoyo elastico
      case 9
        dibujo=crear_dibujo_apoyo('resorte_vertical');
        dibujar_polilineas(dibujo,tamano,modelo.apoyos_beta(n,1),x,2,'k') 
        dibujar_polilineas(dibujo,tamano,pi/2+modelo.apoyos_beta(n,2),x,2,'k') 
        dibujo=crear_dibujo_apoyo('resorte_giro');
        dibujar_polilineas(dibujo,tamano,pi/4+modelo.apoyos_beta(n,3),x,2,'k')
    end
  end

  % Dibujar los nodos de apoyos articulados
  for n=1:modelo.n_apoyos
    x=modelo.nodos_x(modelo.apoyos_nodo(n),:);
    switch modelo.apoyos_tipo(n)
      case {1,2,3,4}
      plot(modelo.nodos_x(:,1)+factor_u*modelo.nodos_u(:,1),...
           modelo.nodos_x(:,2)+factor_u*modelo.nodos_u(:,2),'ok','MarkerSize',6)
    end
  end 

  % Opciones de visualizacion
  title(['Deformada ( x ' sprintf('%9.3e',factor_u) ' )'])
  xlabel 'x_1'
  ylabel 'x_2'
  box on
  axis equal

end

