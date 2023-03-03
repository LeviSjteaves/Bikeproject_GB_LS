clear all;
clc;

% Params
Ts = 0.01;
v = 3; 
h = 0.534;
b = 1.15;
a = 0.6687;
lambda = deg2rad(66);
c = 0.06;
m = 31.3;
g = 9.81;

lr = b; % distance from rear wheel center to center of mass
lf = b-a; % distance from front wheen center to center of mass

% States in global frame
States = [100 10 0.1 0.1 0.1 0.1 3]'; 

% States in local frame
States_l = zeros(7,1);

States_l(1) = States(1) * cos(States(3)) + States(2) * sin(States(3));
States_l(2) = -States(1) * sin(States(3)) + States(2) * cos(States(3));
States_l(3) = 0;        % direction of bike is aligned with X local frame
States_l(4) = States(4);
States_l(5) = States(5);
States_l(6) = States(6);
States_l(7) = States(7);

% Input value (= 0  if A wants to be tested, != if Ax+Bu wants to be
% tested)
dot_delta_e = 0;

%% Matlab script to obtain Kalman gain

% A matrix (linear bicycle model with constant velocity)
A = [0 0 0 0 0 0 1;
     0 0 v 0 0 v*(lr/(lf+lr)) 0;
     0 0 0 0 0 (v/(lr+lf)) 0;
     0 0 0 0 1 0 0;
     0 0 0 (g/h) 0 ((v^2*h-lr*g*c)/(h*(lr+lf))) 0;
     0 0 0 0 0 0 0;
     0 0 0 0 0 0 0];

% B matrix (linear bicycle model with constant velocity)
B = [0 0 0 0 ((lr*v)/(h^2*(lr+lf))) 1 0]';

% C and D matrix (measurement model)
C = eye(7);
D = zeros(7,1);

sys_c = ss(A,B,C,D);
sys_d2 = ss(A,B,C,D,Ts);
sys_d = c2d(sys_c,Ts);
% sys_d.A
% sys_d2.A

% Continuous update
res = A*States_l + B*dot_delta_e

% % Discrete update
% res_d1 = sys_d.A*States_l + sys_d.B*dot_delta_e
% res_d2 = sys_d2.A*States_l + sys_d2.B*dot_delta_e

%% Simulink

% Time update in local frame continuous
states_dot = zeros(7,1);

states_dot(1) = States(7);
states_dot(2) = v * (States_l(3) + (lr/(lr+lf))*States(6) );
states_dot(3) = (v/(lr+lf)) * States(5);
states_dot(4) = States(5);
states_dot(5) = (g/h)*States(4) + ((v^2*h-lr*g*c)/(h*(lr+lf)))*States(6) + ((lr*v)/(h^2*(lr+lf)))*dot_delta_e;
states_dot(6) = dot_delta_e;
states_dot(7) = 0;

% Continuous update
res3 = states_dot

% % Discrete time update
% res_d3 = states_dot*Ts