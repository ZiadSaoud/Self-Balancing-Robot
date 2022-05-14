clear;
clc;
%
g = 9.81;     %Gravitational acceleration
%Pendulum
Mp = 0.96;     %Pendulum weight in kg
Ip = 0.026;   %Pendulum Inertia arround the Y-axis in kg.m2
l = 0.14;    %COG distance

%Wheels
Mw = 0.032;    %wheel weight in kg
Iw = 0.0375; %Wheel Inertia arround the Y-axis in kg.m2(estimated Iw = 0.00428;)
r = 0.0325;   %Wheel radius in m

%DC Motor & Gearbox
Im = 4.59937e-06;%DC motor Rotor Inertia in Kg.m2
Ig = 1.658e-06;  %DC motor Gearbox Inertia in Kg.m2
Bm = 3.1586e-06;   %DC motor damping coefficient in Nms/rad
Bg = 5.7547e-05;   %DC motor Gearbox damping coefficient in Nms/rad
Ng = 4.4;       %DC motor Gearbox ratio
R = 1.261;      %DC motor armature resistance
Kt = 0.012;    %Torque constant
Ke = 0.00806;    %BEMF constant
%state space rep
A=(Mp*r^2)/2 + Mw*r^2 + Iw + Im*Ng^2+Ig;
B=((Bm+Ke*Kt/R)*Ng^2+Bg);
C=((Mp*l*r^2)/2-r*(Im*Ng^2+Ig));
D=r*((Bm+Ke*Kt/R)*Ng^2+Bg);
E=r*Kt*Ng/R;
Ap=Mp*l;
Bp=Mp*l^2+Ip;
Cp=Mp*g*l;
W=[A C;Ap Bp];
Y=[B -D;0 0];
Z=[0 0;0 -Cp];
J=[E;0];
M=[0 0 1 0;0 0 0 1;-inv(W)*Z -inv(W)*Y]
N=[0;0;inv(W)*J]
K=[1 0 0 0;0 1 0 0]
L=[0;0]
