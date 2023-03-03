
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
    % Name of the model
        model = 'Main_v2';
    % Simulation time
        sim_time = 100;
    % Sampling Time
        Ts = 0.01; 
    % Horizon distance [m]
        hor_dis = 1;
    % Constant Speed [m/s]
        v = 3;    

% Open the Simulink Model
    open([model '.slx']);
% Choose the solver
    set_param(model,'AlgebraicLoopSolver','TrustRegion');
% Choose The Bike - Options: 'red' or 'black'
    bike = 'black';
% Load the parameters of the specified bicycle
    bike_params = LoadBikeParameters(bike); 
% bike model
    bike_model = 1; % 1 = non-linear model || 2 = linear model
% Run all test cases
    Run_tests = 0; % 0 = Don't run test cases || 1 = run test cases

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

%% Reference trajectory generation

% SHAPE options: sharp_turn, line, infinite, circle, ascent_sin, smooth_curve
type = 'infinite';
% Distance between points [m]
ref_dis = 0.05;
% Number# of reference points
N = 150; 
% Scale (only for infinite and circle)
scale = 100; 

[Xref,Yref,Psiref] = ReferenceGenerator(type,ref_dis,N,scale);
test_curve=[Xref,Yref,Psiref];
Nn = size(test_curve,1); % needed for simulink

%% Reference test (warnings and initialization update)
Output_reference_test = referenceTest(test_curve,hor_dis,Ts,initial_pose,v, ref_dis);

%update initial states if offset is detected
initial_state.x = Output_reference_test(1);
initial_state.y = Output_reference_test(2);
initial_state.heading = Output_reference_test(3);
initial_pose = [initial_state.x; initial_state.y; initial_state.heading];
initial_state_estimate = initial_state;

%% Disturbance Model
% 
% % Roll Reference  
% roll_ref_generation;%long time ago left by other students, it's helpless now but keep it
% 
% % Steering Rate State Perturbation
% pert_deltadot_state = 0; % Switch ON (1) / OFF (0) the perturbation
% pert_deltadot_state_fun = @(time)  -0.5*(time>10) && (ceil(mod(time/3,2)) == 1) &&(time<30);
% 
% % Roll Rate State Perturbation
% pert_phidot_state = 0; % Switch ON (1) / OFF (0) the perturbation
% pert_phidot_state_fun = @(time) cos(time)*(time>10 && time < 10.4);

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

%% Kalman Filter

% Unpacked bike_params
h = bike_params.h;
b = bike_params.b;
a = bike_params.a;
lambda = bike_params.lambda;
c = bike_params.c;
m = bike_params.m;

% Notations of simulink
lr = bike_params.b; % distance from rear wheel center to center of mass
lf = bike_params.b-bike_params.a; % distance from front wheen center to center of mass

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

% Transform to state space model
% sys = ss(A,B,C,D);   % continuous
% sys_d = ss(A,B,C,D,Ts); % discrete
A_d = (eye(size(A))+Ts*A);

% Q and R matrix
Q = eye(7);
R = eye(7);

% idare function
[P1,Kalman_gain1,eig] = idare(A_d',C',Q,R,[],[]);
eig = abs(eig);
Kalman_gain1 = Kalman_gain1';

% dlqe function
[Kalman_gain2, P2, Z,E] = dlqe(A_d,eye(7),C,Q,R);
Kalman_gain2 = A_d * Kalman_gain2;



%% Start the Simulation
if Run_tests == 0

tic
try Results = sim(model);
    catch error_details %note: the model has runned for one time here
end
toc

%% Simulation Messages and Warnings
if Results.stop.Data(end) == 1
    disp('Message: End of the trajectory has been reached');
end

%% Ploting

% Trajectory
figure();
hold on;
plot(Xref,Yref);
plot(Results.trueX.Data(:,1),Results.trueY.Data(:,1));
plot(Results.predictedX.Data(:,1),Results.predictedY.Data(:,1));
legend('Ref','true','predicted');
xlabel('X-dir [m]');
ylabel('Y-dir [m]');
% ylim([-50 50])
% xlim([-100 100])
grid on;
title('Trajectory');

% X, Y, Psi
figure();

subplot(3,1,1)
hold on;
plot(Results.refX.Time(:,1),Results.refX.Data(:,1));
plot(Results.trueX.Time(:,1),Results.trueX.Data(:,1));
plot(Results.predictedX.Time(:,1),Results.predictedX.Data(:,1));
legend('Xref','trueX','predictedX');
xlabel('Time [t]');
ylabel('Position X [m]');
% ylim([-50 50])
% xlim([-100 100])
grid on;
title('X-coordinate');

subplot(3,1,2);
hold on;
plot(Results.refY.Time(:,1),Results.refY.Data(:,1));                
plot(Results.trueY.Time(:,1),Results.trueY.Data(:,1));              
plot(Results.predictedY.Time(:,1),Results.predictedY.Data(:,1));    
legend('Yref','trueY','predictedY');
xlabel('Time [t]');
ylabel('Position Y [m]');
% ylim([-50 50])
% xlim([-100 100])
grid on;
title('Y-coordinate');

subplot(3,1,3)
hold on;
plot(Results.refPsi.Time(:,1),Results.refPsi.Data(:,1));
plot(Results.truePsi.Time(:,1),Results.truePsi.Data(:,1));
plot(Results.predictedPsi.Time(:,1),Results.predictedPsi.Data(:,1));
legend('Psiref','truePsi','predictedPsi');
xlabel('Time [t]');
ylabel('Angle [rad]');
% ylim([-50 50])
% xlim([-100 100])
grid on;
title('Heading');

% Roll and Roll_rate
figure();

subplot(2,1,1)
hold on;
plot(Results.refRoll.Time(:,1),Results.refRoll.Data(:,1));
plot(Results.trueRoll.Time(:,1),Results.trueRoll.Data(:,1));
plot(Results.predictedRoll.Time(:,1),Results.predictedRoll.Data(:,1));
legend('Rollref','trueRoll','predictedRoll');
xlabel('Time [t]');
ylabel('Angle [rad]');
ylim([-3 3])
% xlim([-100 100])
grid on;
title('Roll');

subplot(2,1,2)
hold on
plot(Results.trueRoll_rate.Time(:,1),Results.trueRoll_rate.Data(:,1));
plot(Results.predictedRoll_rate.Time(:,1),Results.predictedRoll_rate.Data(:,1));
legend('trueRoll rate','predictedRoll rate');
xlabel('Time [t]');
ylabel('Angle rate [rad/s]');
ylim([-3 3])
% xlim([0 0])
grid on;
title('Rollrate');

% Steer angle and rate
figure();

subplot(2,1,1)
hold on;
plot(Results.refSteer_angle.Time(:,1),Results.refSteer_angle.Data(:,1));
plot(Results.trueSteer_angle.Time(:,1),Results.trueSteer_angle.Data(:,1)*sin(bike_params.lambda));
plot(Results.predictedSteer_angle.Time(:,1),Results.predictedSteer_angle.Data(:,1));
legend('refSteer angle','trueSteer angle e','predictedSteer angle e')
xlabel('Time [t]')
ylabel('Angle [rad]')
% ylim([-3 3])
% xlim([0 0])
grid on
title('Steer angle')

subplot(2,1,2)
plot(Results.steer_rate.Time(:,1),Results.steer_rate.Data(:,1))
xlabel('Time [t]')
ylabel('Angle [rad/s]')
% ylim([-3 3])
% xlim([0 0])
grid on
title('Steer rate')

% Ids and closest point index
figure();
hold on
plot(Results.closest_point.Data)
plot(Results.ids.Data)
legend('Closest index', 'ids')
xlabel('Iteration [-]')
ylabel('Index [-]')
% ylim([-3 3])
% xlim([0 0])
grid on
title('Closes+ids')

% Lateral and heading errors
figure();

subplot(1,2,1)
plot(Results.error2.Time,Results.error2.Data) 
xlabel('Iteration')
ylabel('Distance [m]')
grid on
title('Lateral error')

subplot(1,2,2)
plot(Results.error1.Time,Results.error1.Data)
xlabel('Iteration [-]')
ylabel('Angle [rad]')
grid on
title('Heading error')

end

%% Test cases for validation

if Run_tests == 1

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%TEST CASE 1 Sparse infinite
disp('Test case 1: Sparse infinite')

type = 'infinite';
ref_dis = 0.5;
N = 15; 
scale = 100; 
[Xref,Yref,Psiref] = ReferenceGenerator(type,ref_dis,N,scale);
test_curve=[Xref,Yref,Psiref];
Nn = size(test_curve,1); % needed for simulink

%test reference
Output_reference_test = referenceTest(test_curve,hor_dis,Ts,initial_pose,v,ref_dis);

try Results = sim(model);
    catch error_details 
end
% Simulation Messages and Warnings
if Results.stop.Data(end) == 1
    disp('Message: End of the trajectory has been reached');
end

% Trajectory
figure()
plot(Results.refPsi.Time,Results.refPsi.Data(:,1))
figure();
subplot(3,2,[1,3,5]);
hold on;
plot3(Xref,Yref,1:length(Xref));
plot3(Results.trueX.Data(:,1),Results.trueY.Data(:,1),Results.trueY.Time(:,1));
view(0,90)
% plot(Results.predictedX.Data(:,1),Results.predictedY.Data(:,1));
legend('Ref','true');
xlabel('X-dir [m]');
ylabel('Y-dir [m]');
grid on;
title('Trajectory');

subplot(3,2,2)
plot(Results.error2.Time,Results.error2.Data) 
xlabel('Iteration')
ylabel('Distance [m]')
grid on
title('Lateral error')

subplot(3,2,4)
plot(Results.error1.Time,Results.error1.Data)
xlabel('Iteration [-]')
ylabel('Angle [rad]')
grid on
title('Heading error')

subplot(3,2,6)
hold on
plot(Results.closest_point.Time,Results.closest_point.Data)
plot(Results.ids.Time,Results.ids.Data)
legend('Closest index', 'ids')
xlabel('Iteration [-]')
ylabel('Index [-]')
grid on
title('Closes+ids')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %TEST CASE 2 Large offset X:-5 Y:0 PSI: 0
% disp('Test case 2: Large offset')
% 
% type = 'line';
% ref_dis = 0.1;
% N = 1000; 
% scale = 100; 
% [Xref,Yref,Psiref] = ReferenceGenerator(type,ref_dis,N,scale);
% test_curve=[Xref,Yref,Psiref];
% Nn = size(test_curve,1); % needed for simulink
% 
% %test reference
% Output_reference_test = referenceTest(test_curve,hor_dis,Ts,initial_pose,v,ref_dis);
% initial_state.x = Output_reference_test(1);
% initial_state.y = Output_reference_test(2)-5;
% initial_state.heading = Output_reference_test(3);
% initial_pose = [initial_state.x; initial_state.y; initial_state.heading];
% 
% try Results = sim(model);
%     catch error_details 
% end
% % Simulation Messages and Warnings
% if Results.stop.Data(end) == 1
%     disp('Message: End of the trajectory has been reached');
% end
% 
% % Trajectory
% % Trajectory
% figure();
% subplot(3,2,[1,3,5]);
% hold on;
% plot3(Xref,Yref,1:length(Xref));
% plot3(Results.trueX.Data(:,1),Results.trueY.Data(:,1),Results.trueY.Time(:,1));
% view(0,90)
% % plot(Results.predictedX.Data(:,1),Results.predictedY.Data(:,1));
% legend('Ref','true');
% xlabel('X-dir [m]');
% ylabel('Y-dir [m]');
% grid on;
% title('Trajectory');
% 
% subplot(3,2,2)
% plot(Results.error2.Time,Results.error2.Data) 
% xlabel('Iteration')
% ylabel('Distance [m]')
% grid on
% title('Lateral error')
% 
% subplot(3,2,4)
% plot(Results.error1.Time,Results.error1.Data)
% xlabel('Iteration [-]')
% ylabel('Angle [rad]')
% grid on
% title('Heading error')
% 
% subplot(3,2,6)
% hold on
% plot(Results.closest_point.Time,Results.closest_point.Data)
% plot(Results.ids.Time,Results.ids.Data)
% legend('Closest index', 'ids')
% xlabel('Iteration [-]')
% ylabel('Index [-]')
% grid on
% title('Closes+ids')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%TEST CASE 3
% try Results = sim(model);
%     catch error_details 
% end

end


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




