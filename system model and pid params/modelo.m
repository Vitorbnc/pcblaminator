clear all 
close all
clc

s = tf('s');

%Modelo de primeira ordem 

G1 = 333.21/(1+319.28*s)
%step(G1)
%Métricas:
%Ts = 1250s, Erro absoluto = 332
%Polo malha aberta: -1/319.3 =  -0.003132

% Modelo ordem 2
Gp = 332.34/((1+316.03*s)*(1+6.2838*s))
step(Gp)
pzmap(Gp)
%Métricas:
%Ts = 1240s, Erro absoluto = 331
%Polos malha aberta
pole(Gp)
p_plant1= -0.003164;
p_plant2= -0.159139;

%Controlador PID
%Métricas desejadas, especificações realistas
%Eliminar erro em regime permanente-> PI
%Ts aprox = 100s, máx atingível para 100 graus com atuador saturado -> PID
%Max overshoot = 1%, o controle ON/OFF anterior tinha 9%


%PID raiz e simplao por lugar das raízes:
% zeros-> no polos de malha aberta
%Ts = 4/sigma -> sigma = 4/Ts = 4/100 = 0.04
%Um dos pólos fica em zero (integrador)
%Segundo polo, parte real em -0.08, para que o root locus fique na metade
sigma = 0.08;
%Calcular parte imaginaria pelo %UP desejado
os = 0.01;
zeta = abs(log(0.01))/sqrt(pi^2+log(os)^2)
%sigma = real = zeta*w_n
w_n = sigma/zeta
%w_d = imag
w_d = w_n*sqrt(1-zeta^2)

Gc =  (s-p_plant1)*(s-p_plant2)/(s*(s+0.08))

G = Gp*Gc
%pzmap(G) %OK
%1+K*Gc*G = 0, nos polos de malha fechada desejados-> condição de ganho para
%achar o ganho K do controlador
K = abs(evalfr(-1/G,sigma+j*w_d))

Gc_final = K*Gc
Gc_final_factors = zpk(K*Gc)
% 
% syms p_lead Ki Kp Kd S
% Gpid = Kp+Ki/S + (Kd*S)/(S+p_lead);
% Gpid_factor = 1;
% factors = factor(Gpid)
% for N=1:3
%     Gpid_factor = Gpid_factor*factors(N)
% end
% Gpid_factor
% Gc_final
%plead = 0.08;
%Ki = 4.926e-5/plead;

K = 4.926e-05;
Ti = 0.08;
Td = 0.09783/(K*Ti)
alpha = 1/(Ti*Td)






