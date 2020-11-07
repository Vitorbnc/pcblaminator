
s = tf('s');
%Modelo ordem 3
Gp = 292.61/((1+218.89*s)*(1+30.713*s)*(1+28.38*s))
subplot(2,2,1)
step(Gp)

Tplanta = feedback(Gp,1)
subplot(2,2,3)
step(Tplanta)

%Controlador PID
%Métricas desejadas, especificações realistas
%Eliminar erro em regime permanente-> PI
%Ts aprox = 150s, máx atingível para 100 graus com atuador saturado -> PID
%Max overshoot = 1%, o controle ON/OFF tem cerca de 11%


%PID raiz por lugar das raízes:
% zeros-> no polos de malha aberta
%Ts = 4/sigma -> sigma = 4/Ts
%Um dos pólos fica em zero (integrador)
%Segundo polo, parte real em 2*sigma, para que o root locus fique na metade

plant_poles = pole(Gp)
pzmap(Gp)
Gc = (s-plant_poles(1))*(s-plant_poles(3))/s

rlocus(Gp*Gc)
%Ganho escolhido pelo lugar das raízes, maior ganho com mínimo de overshoot (0.7%)
K = 0.205
Gc = K*Gc

T = feedback(Gp*Gc,1)
subplot(2,2,2)
step(T)

%Comparando coeficientes com PID canônico
%syms Kp Ki Kd S

%Gpid = Kp+Kd*S+Ki/S
%Gpid = (Kd*S^2+Kp*S+Ki)/S

Kd = 0.205;
Kp = 0.00816;
Ki = 3.3e-5;

