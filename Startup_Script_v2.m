
%% clear the possible remnant on previous running

set(0,'defaulttextinterpreter','none');
dbclear all;
clear;
close all;
clc;

%% Simulation Settings and Bike Parameters
% General Parameters

    % Gravitational Acceleration
        g = 9.81;

    % Simulation time
        sim_time = 150;

    % Name of the model
        model = 'Main_v2';

    % Sampling Time
        Ts = 0.01; 

    % Horizon time
        Th = 0.02;

% Initial states

    % Initial Roll
        initial_state.roll = deg2rad(0);
        initial_state.roll_rate = deg2rad(0);

    % Initial Steering
        initial_state.steering = deg2rad(0);

    % Initial Pose (X,Y,\theta)
        initial_state.x = 0;
        initial_state.y = 0;
        initial_state.heading = deg2rad(0);
        initial_pose = [initial_state.x; initial_state.y; initial_state.heading];

% Constant Speed [m/s]
    v = 3;    

% Open the Simulink Model
    open([model '.slx']);
% Choose the solver
    set_param(model,'AlgebraicLoopSolver','TrustRegion');
% Choose The Bike - Options: 'red' or 'black'
    bike = 'black';
% bike model
    bike_model = 1; % 1 = non-linear model || 2 = linear model

% Load the parameters of the specified bicycle
    bike_params = LoadBikeParameters(bike); 

%% Disturbance Model

% Roll Reference  
roll_ref_generation;%long time ago left by other students, it's helpless now but keep it

% Steering Rate State Perturbation
pert_deltadot_state = 0; % Switch ON (1) / OFF (0) the perturbation
pert_deltadot_state_fun = @(time)  -0.5*(time>10) && (ceil(mod(time/3,2)) == 1) &&(time<30);

% Roll Rate State Perturbation
pert_phidot_state = 0; % Switch ON (1) / OFF (0) the perturbation
pert_phidot_state_fun = @(time) cos(time)*(time>10 && time < 10.4);

%% Bike State-Space Model

% Continuous-Time Model

% % Controllable Canonical Form
%     A = [0 g/bike_params.h ; 1 0];
%     B = [1 ; 0];
%     C = [bike_params.a*v/(bike_params.b*bike_params.h) g*bike_params.inertia_front/(bike_params.h^3*bike_params.m)+v^2/(bike_params.b*bike_params.h)];
%     D = [bike_params.inertia_front/(bike_params.h^2*bike_params.m)];

% Observable Canonical Form
    A = [0 g/bike_params.h ; 1 0];
    B = [g*bike_params.inertia_front/(bike_params.h^3*bike_params.m)+(v^2./(bike_params.b*bike_params.h)-bike_params.a*bike_params.c*g/(bike_params.b*bike_params.h^2)).*sin(bike_params.lambda) ;
        bike_params.a*v/(bike_params.b*bike_params.h).*sin(bike_params.lambda)];
    C = [0 1];
    D = [bike_params.inertia_front/(bike_params.h^2*bike_params.m)];

% Linearized System
    linearized_sys = ss(A,B,C,D);
% Augmented System
    fullstate_sys = ss(linearized_sys.A,linearized_sys.B,eye(size(linearized_sys.A)),0);
% Discretized System
    discretized_sys = c2d(linearized_sys,Ts);

%% Balancing Controller

% Outer loop -- Roll Tracking
P_balancing_outer = 1.3;
I_balancing_outer = 0.0;
D_balancing_outer = 0.0;

% Inner loop -- Balancing
P_balancing_inner = 3;
I_balancing_inner = 0;
D_balancing_inner = 0;  

%% The LQR controller

%error model for LQR controller calculation
A_con=[0 v;0 0];
B_con=[bike_params.a*v/bike_params.b;v/bike_params.b];

 kk=0.000000010;
 Q=kk*[1000 0;0 100];
 R=0.5;
 [K,S,e] = lqr(A_con,B_con,Q,R);
 k1=K(1);
 k2=K(2);

e2_max=deg2rad(30);%Here is the e2_max we used to calculate e1_max
e1_max=abs(-k2*e2_max/k1);% k1,k2 has been known, so we can calculate e1_max

%% Reference trajectory generation

Ts_ref = 10*Ts; % Sampling time for reference generation
N = 50; % # of reference points
scale = 100; % only for infinite and circle

% options: sharp_turn, line, infinite, circle, ascent_sin, smooth_curve
type = 'infinite';

[Xref,Yref,Psiref] = ReferenceGenerator(type,Ts_ref,N,scale);
test_curve=[Xref,Yref,Psiref];
Nn = size(test_curve,1); % needed for simulink

%% Warnings

initial_pose_new = referenceTest(test_curve,Th,Ts,initial_pose);
initial_state.x = initial_pose_new(1);
initial_state.y = initial_pose_new(2);
initial_state.heading = initial_pose_new(3);
initial_pose = [initial_state.x; initial_state.y; initial_state.heading];

%% Start the Simulation

tic
try Results = sim(model);
    catch error_details %note: the model has runned for one time here
end
toc

%% Messages
if Results.stop.Data(end) == 1
    disp('Message: End of the trajectory has been reached');
end

%% Ploting

% Trajectory
figure();
plot(Xref,Yref);
hold on;
plot(Results.trueX.Data(:,1),Results.trueY.Data(:,1));
legend('Ref','true');
xlabel('X-dir');
ylabel('Y-dir');
grid on;
title('Trajectory');

% X, Y, Psi
figure();
subplot(3,1,1)
plot(Results.refX.Time(:,1),Results.refX.Data(:,1));
hold on;
plot(Results.trueX.Time(:,1),Results.trueX.Data(:,1));
legend('Xref','trueX');
xlabel('Time [t]');
ylabel('Position X [m]');
grid on;
title('X-coordinate');
subplot(3,1,2);
plot(Results.refY.Time(:,1),Results.refY.Data(:,1));
hold on;
plot(Results.trueY.Time(:,1),Results.trueY.Data(:,1));
legend('Yref','trueY');
xlabel('Time [t]');
ylabel('Position Y [m]');
grid on;
title('Y-coordinate');
subplot(3,1,3)
plot(Results.refPsi.Time(:,1),Results.refPsi.Data(:,1));
hold on;
plot(Results.truePsi.Time(:,1),Results.truePsi.Data(:,1));
legend('Psiref','truePsi');
xlabel('Time [t]');
ylabel('Angle');
grid on;
title('Heading');

% Roll and Roll_rate
figure();
subplot(2,1,1)
plot(Results.refRoll.Time(:,1),Results.refRoll.Data(:,1));
hold on;
plot(Results.trueRoll.Time(:,1),Results.trueRoll.Data(:,1));
legend('Rollref','trueRoll');
xlabel('Time [t]');
ylabel('Angle');
grid on;
title('Roll');
subplot(2,1,2)
plot(Results.trueRoll_rate.Time(:,1),Results.trueRoll_rate.Data(:,1));
xlabel('Time [t]');
ylabel('Angle');
grid on;
title('Roll rate');

% Steer angle and rate
figure();
subplot(2,1,1)
plot(Results.steer_angle.Time(:,1),Results.steer_angle.Data(:,1)) 
xlabel('Time')
ylabel('Angle')
title('Steer angle')
subplot(2,1,2)
plot(Results.steer_rate.Time(:,1),Results.steer_rate.Data(:,1))
xlabel('Time [t]')
ylabel('Angle/s')
title('Steer rate')

% Ids and closest point index
figure();
hold on
plot(Results.closest_point.Data)
plot(Results.ids.Data)
title('Closes+ids')
legend('Closest index', 'ids')

% Lateral and heading errors
figure();
subplot(1,2,1)
plot(Results.error2.Time,Results.error2.Data) 
xlabel('Iteration')
ylabel('Distance [m]')
title('Lateral error')
subplot(1,2,2)
plot(Results.error1.Time,Results.error1.Data)
xlabel('Iteration')
ylabel('Degree')
title('Heading error')

%% Utility Functions

function Parameters = LoadBikeParameters(bike)
    if strcmp(bike,'red')
        % Red bike
        Parameters.inertia_front = 0.245;  %[kg.m^2] inertia of the front wheel
        Parameters.r_wheel = 0.311;        % radius of the wheel
        Parameters.h = 0.2085 + Parameters.r_wheel;   % height of center of mass [m]
        Parameters.b = 1.095;              % length between wheel centers [m]
        Parameters.c = 0.06;               % length between front wheel contact point and the extention of the fork axis [m]
        Parameters.lambda = deg2rad(70);   % angle of the fork axis [deg]
        Parameters.a = 0.4964;             % distance from rear wheel to frame's center of mass [m]
        Parameters.IMU_height = 0.45;      % IMU height [m]
        Parameters.m = 45;                 % Bike mas [kg]
        Parameters.uneven_mass = false;    % true = use uneven mass distribution in bike model ; 0 = do not use
    elseif strcmp(bike,'black')
        % Black bike
        Parameters.inertia_front = 0.245;  %[kg.m^2] inertia of the front wheel
        Parameters.h = 0.534 ;             % height of center of mass [m]
        Parameters.b = 1.15;               % length between wheel centers [m]
        Parameters.a = 0.6687;             % distance from rear wheel to frame's center of mass [m]
        Parameters.c = 0.06;               % length between front wheel contact point and the extention of the fork axis [m]
        Parameters.IMU_height = 0.215;     % IMU height [m]
        Parameters.m = 31.3;               % Bike mass [kg]
        Parameters.lambda = deg2rad(66);   % angle of the fork axis [deg]  !!! TO BE MEASURED
        Parameters.uneven_mass = false;    % true = use uneven mass distribution in bike model ; 0 = do not use
    end
end


