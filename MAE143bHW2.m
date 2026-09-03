% Diego Meza
% MAE143b Prof. Beweley
% September 2, 2026
% HW 2

close all;
clear;
clc;

%% Set up Renaissance Robotics path
RRbase = 'C:\Users\diego\Downloads\RR-main\RR-main';

addpath(genpath(RRbase));
rehash toolboxcache;

d = 12;
a0 = 0.02

F_2_2 = RR_pade(d,2,2); %Low-order pade approx
G = F_2_2*RR_tf(1,[1/a0 1]); %plant
D = 1;
P = 1/0.5;
figure(1)
RR_rlocus(G)
axis([-.4 .3 -.3 .3])
title('Root Locus')

figure(2)
g.T = 200;
RR_step(35 + 10*P*G*D/(1+G*D),g)
axis([0 200 32 55])
title('Temperature Response with Proportional Control')

figure(3)
RR_step(35 + 10*P*D/(1+G*D),g)
axis([0 200 40 60])
title('Control Input with Proportional Control')

Kp = 1;
Ki = 0.025;
D2 = RR_tf([Kp Ki],[1 0]);

figure(4)
g.T = 450;
RR_step(35 + 10*G*D2/(1+G*D2),g)
axis([0 450 32 50])
title('Temperature Response: PI Control, F_{2,2}')

figure(5)
RR_step(35 + 10*D2/(1+G*D2),g)
axis([0 450 5 55])
hold on
plot([0 450],[50 50],'r--')
plot([0 450],[10 10],'r--')
hold off
title('Control Input: PI Control, F_{2,2}')

%verrification
F_16_13 = RR_pade(d,16,13);
G_hi = F_16_13*RR_tf(1,[1/a0 1]);

figure(6)
g.T = 450;
RR_step(35 + 10*G_hi*D2/(1+G_hi*D2),g)
axis([0 450 32 50])
title('Temperature Response: PI Control, F_{16,13}')

figure(7)
RR_step(35 + 10*D2/(1+G_hi*D2),g)
axis([0 450 5 55])
hold on
plot([0 450],[50 50],'r--')
plot([0 450],[10 10],'r--')
hold off
title('Control Input: PI Control, F_{16,13}')