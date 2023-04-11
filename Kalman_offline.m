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
        model = 'Kalman_offline_sim';
    % Sampling Time
        Ts = 0.01; 
% Open the Simulink Model
    open([model '.slx']);
% Choose the solver
    set_param(model,'AlgebraicLoopSolver','TrustRegion');

    
%%%%%%%%%%%%%GUI%%%%%%%%%%%%%%%%%
% reduce/increase simulation time for desired timescale on x-axis of the plots
sim_time = 35;

% 0 for Yixiao's measurement data, 1 for measurement data which is recorded during a simulation (infinity shape)
select = 0; 

% Take into account a valid speed. 
% Yixiao's measurements: 0-2 m/s.
% Simulation measurements: 3 m/s
v=2;

% Choose The Bike - Options: 'red' or 'black' 
% Yixiao uses black 
% Simulation uses red
    bike = 'black';

% Load the parameters of the specified bicycle
    bike_params = LoadBikeParameters(bike); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Unpacked bike_params

h = bike_params.h_mod;
lr = bike_params.lr_mod;
lf = bike_params.lf_mod; 
lambda = bike_params.lambda_mod;
c = bike_params.c_mod;
m = bike_params.m_mod;
h_imu = bike_params.IMU_height_mod;

% Correction IMU
IMU_x = bike_params.IMU_x_mod;
IMU_roll = bike_params.IMU_roll_mod;
IMU_pitch = bike_params.IMU_pitch_mod;            
IMU_yaw = bike_params.IMU_yaw_mod;

% Convert orientation offsets to radians
roll = IMU_roll * pi / 180;
pitch = IMU_pitch * pi / 180;
yaw = IMU_yaw * pi / 180;

% Calculate transformation matrix
Rx = [1 0 0; 0 cos(roll) -sin(roll); 0 sin(roll) cos(roll)];   % Rotation matrix for roll
Ry = [cos(pitch) 0 sin(pitch); 0 1 0; -sin(pitch) 0 cos(pitch)]; % Rotation matrix for pitch
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];        % Rotation matrix for yaw

T = Rz*Ry*Rx; 


%% Load measurements
% BikeData_Yixiao
csv_name = 'BikeData_20230222-205544.csv';   data_range = [7, 34.1902]; %  tstvagain v24 3 seg step0
CSV_With_EXCEPTION = 1;

if CSV_With_EXCEPTION == 1
    data1 = readtable(csv_name,'headerlines',1); 
    data1_len = size(data1,1);
    opts = detectImportOptions(csv_name);
    opts.DataLines = [4 data1_len]; % remove the last row, as it would be the termination flag
    data1 = readtable(csv_name,opts);
    fid = fopen(csv_name);
    csv_Data = textscan(fid, '%[^\n]', data1_len+2, 'HeaderLines', 0);
    csv_Data = csv_Data{1};
    ExitFlag = split(csv_Data{data1_len+1}, ',');
    ExitData = split(csv_Data{data1_len+2}, ',');
     
else
    data1 = readtable(csv_name,'headerlines',1);
end

% BikeData_GaizkaLevi (infinity shape)
data2 = readtable('bikedata_simulation_measurements.csv');
% steer_rate_simulation = [table2array(data2(:,1)) table2array(data2(:,9))];
Kalman_filter_simulation = table2array(data2(:,1:8));

data3 = readtable('bikedata_simulation_bikestates.csv');
data4 = readtable('bikedata_simulation_estimation.csv');

%% Initial states
if select == 1
% Initial Roll
        initial_state.roll = deg2rad(0);
        initial_state.roll_rate = deg2rad(0);
% Initial Steering
        initial_state.steering = deg2rad(0);
% Initial Pose (X,Y,theta)
        initial_state.x = data3.Var2(1);
        initial_state.y = data3.Var3(1);
        initial_state.heading = deg2rad(data3.Var4(1));
        initial_pose = [initial_state.x; initial_state.y; initial_state.heading];
        initial_state_estimate = initial_state;
else
% Initial Roll
        initial_state.roll = deg2rad(0);
        initial_state.roll_rate = deg2rad(0);
% Initial Steering
        initial_state.steering = deg2rad(0);
% Initial Pose (X,Y,theta)
        initial_state.x = 0;
        initial_state.y = 0;
        initial_state.heading = deg2rad(0);
        initial_pose = [initial_state.x; initial_state.y; initial_state.heading];
        initial_state_estimate = initial_state;
end


%% Select right measurements and input
% Converting GPS data to X and Y position
% Setting X and Y to zero at first long/lat datapoint
longitude0 = deg2rad(data1.longitude(2));
latitude0 = deg2rad(data1.latitude(2));
Earth_rad = 6371000.0;

X = Earth_rad * (deg2rad(data1.longitude) - longitude0) * cos(latitude0);
Y = Earth_rad * (deg2rad(data1.latitude) - latitude0);

ay = data1.ay;
omega_x = data1.RollRate;
omega_y = data1.gy;
delta_enc = data1.SteeringAngle - data1.steering_offset;
v_enc = data1.FilteredVelocity;

steer_rate_simulation = [data1.Time data1.SteeringAngle];

% Prepare measurement data for the offline kalman
measurementsGPS = [data1.GPS_timestamp X Y];
measurementsGPS(1,:) = [];
measurements = [data1.Time ay omega_x omega_y delta_enc v_enc];
measurements(1,:) = [];
steer_rate_blackbike = [data1.Time data1.SteeringAngle];
steer_rate_blackbike(1,:) = [];


%% Kalman Filter

% A matrix (linear bicycle model with constant velocity)
% Est_States := [X Y psi phi phi_dot delta v]
A = [0 0 0 0 0 0 1;
     0 0 v 0 0 v*(lr/(lf+lr))*sin(lambda) 0;
     0 0 0 0 0 (v/(lr+lf))*sin(lambda) 0;
     0 0 0 0 1 0 0;
     0 0 0 (g/h) 0 ((v^2*h-lr*g*c)/(h^2*(lr+lf)))*sin(lambda) 0;
     0 0 0 0 0 0 0;
     0 0 0 0 0 0 0];

% B matrix (linear bicycle model with constant velocity)
B = [0 0 0 0 ((lr*v)/(h*(lr+lf))) 1 0]';

% Including GPS
C1 = [1 0 0 0 0 0 0;
     0 1 0 0 0 0 0;
     0 0 0 -g+((-h_imu*g)/h) 0 (-h_imu*(h*v^2-(g*lr*c)))*sin(lambda)/((lr+lf)*h^2) + (v^2)*sin(lambda)/(lr+lf) 0;
     0 0 0 0 1 0 0;
     0 0 0 0 0 (v)*sin(lambda)/(lr+lf) 0;
     0 0 0 0 0 1 0;
     0 0 0 0 0 0 1];

D1 = [0 0 (-h_imu*lr*v)/((lr+lf)*h) 0 0 0 0]';

% Excluding GPS
C2 = [-g+((-h_imu*g)/h) 0 (-h_imu*(h*v^2-(g*lr*c)))*sin(lambda)/((lr+lf)*h^2) + (v^2)*sin(lambda)/(lr+lf) 0;
      0 1 0 0;
      0 0 (v)*sin(lambda)/(lr+lf) 0;
      0 0 1 0;
      0 0 0 1];

D2 = [(-h_imu*lr*v)/((lr+lf)*h) 0 0 0 0]';

% Discretize the model
A_d = (eye(size(A))+Ts*A);
B_d = Ts*B;

% Q and R matrix
Q = eye(7);
Q2 = eye(4);
R1 = eye(7);
R2 = eye(5);

% Compute Kalman Gain
    % including GPS
    [P1,Kalman_gain1,eig] = idare(A_d',C1',Q,R1,[],[]);
    eig1 = abs(eig);
    Kalman_gain1 = Kalman_gain1';
    Ts_GPS = 0.1; % sampling rate of the GPS
    counter = (Ts_GPS / Ts) - 1 ; % Upper limit of the counter for activating the flag

    % excluding GPS
    A_d2 = A_d;
    A_d2(1:3,:) = [];
    A_d2(:,1:3) = [];
    [P2,Kalman_gain2,eig] = idare(A_d2',C2',Q2,R2,[],[]);
    eig2 = abs(eig);
    Kalman_gain2 = Kalman_gain2';

% Polish the kalman gain (values <10-5 are set to zero)
for i = 1:size(Kalman_gain1,1)
    for j = 1:size(Kalman_gain1,2)
        if abs(Kalman_gain1(i,j)) < 10^-5
            Kalman_gain1(i,j) = 0;
        end
    end
end 
for i = 1:size(Kalman_gain2,1)
    for j = 1:size(Kalman_gain2,2)
        if abs(Kalman_gain2(i,j)) < 10^-5
            Kalman_gain2(i,j) = 0;
        end
    end
end

 %% Start the Simulation

tic
try Results = sim(model);
    catch error_details %note: the model has runned for one time here
end
toc

%% Comparing different Q R
% Q = eye(7);
% Q2(4,4)=0.5;
Qgain = 1;
Rgain = 1;
% Parameters of Q
Q_GPS = 0.2^2;
Q_Psi = 1^2;
Q_roll = deg2rad(0.1)^2;
Q_rollrate = deg2rad(0.05)^2;
Q_delta = deg2rad(0.05)^2;
Q_v = 0.1^2;
Qscale = 1;
Q =Qscale* [Q_GPS 0 0 0 0 0 0;
              0 Q_GPS 0 0 0 0 0;
              0 0 Q_Psi 0 0 0 0;
              0 0 0 Q_roll 0 0 0;
              0 0 0 0 Q_rollrate 0 0;
              0 0 0 0 0 Q_delta 0;
              0 0 0 0 0 0 Q_v];

Q2 =Qscale* [Q_roll 0 0 0;
               0 Q_rollrate 0 0;
               0 0 Q_delta 0;
               0 0 0 Q_v];

% Parameters of R
R_GPS = 0.2^2;
R_ay =deg2rad(0.1)^2;
R_wx = deg2rad(0.1)^2;
R_wz = deg2rad(0.1)^2;
R_delta = 0.001^2;
R_v = 10^2;
Rscale = 1;
R1 =Rscale* [R_GPS 0 0 0 0 0 0;
              0 R_GPS 0 0 0 0 0;
              0 0 R_ay 0 0 0 0;
              0 0 0 R_wx 0 0 0;
              0 0 0 0 R_wz 0 0;
              0 0 0 0 0 R_delta 0;
              0 0 0 0 0 0 R_v];

R2 =Rscale* [R_ay 0 0 0 0;
              0 R_wx 0 0 0;
              0 0 R_wz 0 0;
              0 0 0 R_delta 0;
              0 0 0 0 R_v];

% Compute Kalman Gain
    % including GPS
    [P1,Kalman_gain1,eig] = idare(A_d',C1',Q,R1,[],[]);
    eig1 = abs(eig);
    Kalman_gain1 = Kalman_gain1';
    Ts_GPS = 0.1; % sampling rate of the GPS
    counter = (Ts_GPS / Ts) - 1 ; % Upper limit of the counter for activating the flag

    % excluding GPS
    A_d2 = A_d;
    A_d2(1:3,:) = [];
    A_d2(:,1:3) = [];
    [P2,Kalman_gain2,eig] = idare(A_d2',C2',Q2,R2,[],[]);
    eig2 = abs(eig);
    Kalman_gain2 = Kalman_gain2';   
%  Kalman_gain1(4,:)=0;
%  Kalman_gain2(2,:)=0;

% Polish the kalman gain (values <10-5 are set to zero)
for i = 1:size(Kalman_gain1,1)
    for j = 1:size(Kalman_gain1,2)
        if abs(Kalman_gain1(i,j)) < 10^-5
            Kalman_gain1(i,j) = 0;
        end
    end
end 
for i = 1:size(Kalman_gain2,1)
    for j = 1:size(Kalman_gain2,2)
        if abs(Kalman_gain2(i,j)) < 10^-5
            Kalman_gain2(i,j) = 0;
        end
    end
end

tic
try Results2 = sim(model);
    catch error_details %note: the model has runned for one time here
end
toc

%% Plot results
close all

%GPS correction Yixiao
alp = deg2rad(0);
GPSXY = [data1.x_estimated data1.y_estimated];
RotGPS = [cos(alp) sin(alp); -sin(alp) cos(alp)];
GPS_estimated_cor = GPSXY*RotGPS ;

if select == 0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fig = figure();
plot(Results.sim_Kalman.Data(:,1), Results.sim_Kalman.Data(:,2))
hold on
plot(GPS_estimated_cor(:,1),GPS_estimated_cor(:,2))
plot(measurementsGPS(:,2),measurementsGPS(:,3))
xlabel('X position (m)')
ylabel('Y position (m)')
grid on
legend('offline Kalman estimation', 'Yixiao estimation','GPS measurements')
title('Comparison with Yixiao measurement data')

fig = figure();
subplot(421)
plot(Results.sim_Kalman.Time,Results.sim_Kalman.Data(:,1))
hold on
plot(Results2.sim_Kalman.Time,Results2.sim_Kalman.Data(:,1))
plot(data1.Time, GPS_estimated_cor(:,1))
plot(measurementsGPS(:,1),measurementsGPS(:,2))
xlabel('Time (s)')
ylabel('X position (m)')
grid on
legend('offline Kalman estimation','offline Kalman estimation Tuned R', 'Yixiao estimation','GPS measurements')
title('Comparison with Yixiao measurement data')

subplot(423)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,2))
hold on
plot(Results2.sim_Kalman.Time, Results2.sim_Kalman.Data(:,2))
plot(data1.Time, GPS_estimated_cor(:,2))
plot(measurementsGPS(:,1),measurementsGPS(:,3))
xlabel('Time (s)')
ylabel('Y position (m)')
grid on
legend('offline Kalman estimation','offline Kalman estimation Tuned R', 'Yixiao estimation','GPS measurements')

subplot(425)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,3))
hold on
plot(Results2.sim_Kalman.Time, Results2.sim_Kalman.Data(:,3))
plot(data1.Time, data1.yaw_estimated+alp)
xlabel('Time (s)')
ylabel('heading (rad)')
grid on
legend('offline Kalman estimation','offline Kalman estimation Tuned R', 'Yixiao estimation')

subplot(422)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,4))
hold on
plot(Results2.sim_Kalman.Time, Results2.sim_Kalman.Data(:,4))
plot(data1.Time,data1.Roll)
plot(Results2.integration_rollrate.Time, Results2.integration_rollrate.Data)
xlabel('Time (s)')
ylabel('Roll (rad)')
grid on
legend('offline Kalman estimation','offline Kalman estimation Tuned R', 'Yixiao estimation','Integration Rollrate')
title('Comparison with Yixiao measurement data')

subplot(424)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,5))
hold on
plot(Results2.sim_Kalman.Time, Results2.sim_Kalman.Data(:,5))
plot(data1.Time, data1.RollRate)
xlabel('Time (s)')
ylabel('Roll Rate (rad/s)')
grid on
legend('offline Kalman estimation','offline Kalman estimation Tuned R', 'Yixiao estimation')

subplot(426)
plot(Results.sim_Kalman.Time,Results.sim_Kalman.Data(:,6))
hold on
plot(Results2.sim_Kalman.Time,Results2.sim_Kalman.Data(:,6))
plot(data1.Time,data1.SteeringAngle - data1.steering_offset)
xlabel('Time (s)')
ylabel('Steering Angle (rad)')
grid on
legend('offline Kalman estimation','offline Kalman estimation Tuned R', 'Yixiao estimation')


% subplot(427)
% plot(Results.y_hat.Time,Results.y_hat.Data)
% hold on
% plot(Results2.integration_steerrate)
% xlabel('Time (s)')
% ylabel('velocity (m/s)')
% grid on
% legend('offline Kalman estimation','offline Kalman estimation Tuned R', 'Yixiao estimation')

subplot(428)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,7))
hold on
plot(Results2.sim_Kalman.Time, Results2.sim_Kalman.Data(:,7))
plot(data1.Time, data1.v_estimated)
xlabel('Time (s)')
ylabel('velocity (m/s)')
grid on
legend('offline Kalman estimation','offline Kalman estimation Tuned R', 'Yixiao estimation')

% figure
% subplot(221)
% plot(Results.y_hat.Time,Results.y_hat.Data(:,1))
% hold on
% plot(measurements(:,1),measurements(:,2))
% xlabel('Time (s)')
% ylabel(' (m/s^2)')
% title('a_y')
% grid on
% legend('prediction','meas')
% subplot(222)
% plot(Results.y_hat.Time,Results.y_hat.Data(:,2))
% hold on
% plot(measurements(:,1),measurements(:,3))
% xlabel('Time (s)')
% ylabel(' (rad/s)')
% title('w_x')
% grid on
% legend('prediction','meas')
% subplot(223)
% plot(Results.y_hat.Time,Results.y_hat.Data(:,3))
% hold on
% plot(measurements(:,1),measurements(:,4))
% xlabel('Time (s)')
% ylabel(' (rad/s)')
% title('w_z')
% grid on
% legend('prediction','meas')
% subplot(224)
% plot(Results.y_hat.Time,Results.y_hat.Data(:,4))
% hold on
% plot(measurements(:,1),measurements(:,5))
% xlabel('Time (s)')
% ylabel(' (rad)')
% title('delta_enc')
% grid on
% legend('prediction','meas')

% figure
% plot(Results.sim_Kalman.Time,Results.sim_Kalman.Data(:,6))
% hold on
% plot(Results2.sim_Kalman.Time,Results2.sim_Kalman.Data(:,6))
% plot(data1.Time,data1.SteeringAngle - data1.steering_offset)
% plot(data1.Time,data1.SteeringAngle)
% plot(data1.Time,steer_rate_calc)
% xlabel('Time (s)')
% ylabel('Steering Angle (rad)')
% grid on
% legend('offline Kalman estimation','offline Kalman estimation Tuned R', 'Yixiao estimation','Measured steering angle','Diff steerangle')


else
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fig = figure()
plot(Results.sim_Kalman.Data(:,1), Results.sim_Kalman.Data(:,2))
hold on
plot(data3.Var2,data3.Var3)
plot(Kalman_filter_simulation(:,2),Kalman_filter_simulation(:,3))
xlabel('X position (m)')
ylabel('Y position (m)')
grid on
legend('offline Kalman estimation', 'True state','Measurements','GPS measurements')
title('Comparison with simulation measurement data')

fig = figure()
subplot(421)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,1))
hold on
plot(data3.Var1, data3.Var2)
plot(Kalman_filter_simulation(:,1),Kalman_filter_simulation(:,2))
xlabel('Time (s)')
ylabel('X position (m)')
grid on
legend('offline Kalman estimation', 'True state','GPS measurements')
title('Comparison with simulation measurement data')

subplot(423)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,2))
hold on
plot(data3.Var1, data3.Var3)
plot(Kalman_filter_simulation(:,1),Kalman_filter_simulation(:,3))
xlabel('Time (s)')
ylabel('Y position (m)')
grid on
legend('offline Kalman estimation', 'True state','GPS measurements')

subplot(425)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,3))
hold on
plot(data3.Var1, data3.Var4)
xlabel('Time (s)')
ylabel('heading (rad)')
grid on
legend('offline Kalman estimation', 'True state')

subplot(422)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,4))
hold on
plot(data3.Var1, data3.Var5)
xlabel('Time (s)')
ylabel('Roll (rad)')
grid on
legend('offline Kalman estimation', 'True state')
title('Comparison with simulation measurement data')

subplot(424)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,5))
hold on
plot(data3.Var1, data3.Var6)
xlabel('Time (s)')
ylabel('Roll Rate (rad/s)')
grid on
legend('offline Kalman estimation', 'True state')

subplot(426)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,6))
hold on
plot(data3.Var1, data3.Var7)
xlabel('Time (s)')
ylabel('Steering Angle (rad)')
grid on
legend('offline Kalman estimation', 'True state')

subplot(428)
plot(Results.sim_Kalman.Time, Results.sim_Kalman.Data(:,7))
hold on
plot(data3.Var1, data3.Var8)
xlabel('Time (s)')
ylabel('Velocity (m/s)')
grid on
legend('offline Kalman estimation', 'True state')

end

%% Utility Functions

function Parameters = LoadBikeParameters(bike)

    if strcmp(bike,'red')
        % Red bike
 % real parameters and positions on bike
        Parameters.inertia_front = 0.245;  %[kg.m^2] inertia of the front wheel
        Parameters.r_wheel = 0.311;        % radius of the wheel
        Parameters.h = 0.2085 + Parameters.r_wheel;   % height of center of mass [m]
        Parameters.lr = 0.4964;             % distance from rear wheel to frame's center of mass [m]
        Parameters.lf = 1.095-0.4964;       % distance from front wheel to frame's center of mass [m]
        Parameters.c = 0.06;               % length between front wheel contact point and the extention of the fork axis [m]
        Parameters.m = 45;                 % Bike mas [kg]
        Parameters.lambda = deg2rad(70);   % angle of the fork axis [deg]
        Parameters.IMU_height = 0.615;      % IMU height [m]
        Parameters.IMU_x = 0.0;           % x Position of the IMU measured from rear wheel (parallel to bike) [m]
        Parameters.IMU_roll = 0;           % Orientation offset in roll (degrees)
        Parameters.IMU_pitch = 0;            % Orientation offset in pitch (degrees)
        Parameters.IMU_yaw = 0;             % Orientation offset in yaw (degrees)
        Parameters.Xgps = 0.0;             % Actual GPS position offset X (measured from middlepoint position parralel to bike heading) [m]
        Parameters.Ygps = 0.0;             % Actual GPS position offset Y (measured from middlepoint position perpendicular to bike heading) [m]
        Parameters.Hgps = 0.0;             % GPS position height (measured from the groud to the GPS)   [m]
        
        % Parameters in the model
        Parameters.inertia_front_mod = 0.245;  %[kg.m^2] inertia of the front wheel
        Parameters.r_wheel_mod = 0.311;        % radius of the wheel
        Parameters.h_mod = 0.2085 + Parameters.r_wheel_mod;   % height of center of mass [m]
        Parameters.lr_mod = 0.4964;             % distance from rear wheel to frame's center of mass [m]
        Parameters.lf_mod = 1.095-0.4964;       % distance from front wheel to frame's center of mass [m]
        Parameters.c_mod = 0.06;                % length between front wheel contact point and the extention of the fork axis [m]
        Parameters.m_mod = 45;                  % Bike mas [kg]
        Parameters.lambda_mod = deg2rad(70);    % angle of the fork axis [deg]
        Parameters.IMU_height_mod = 0.615;      % IMU height [m]
        Parameters.IMU_x_mod = 0.0;           % x Position of the IMU measured from rear wheel (parallel to bike) [m]
        Parameters.IMU_roll_mod = 0;           % Orientation offset in roll (degrees)
        Parameters.IMU_pitch_mod = 0;            % Orientation offset in pitch (degrees)
        Parameters.IMU_yaw_mod = 0;             % Orientation offset in yaw (degrees)
        Parameters.Xgps_mod = 0.0;              % GPS position X accoarding to the model [m] 
        Parameters.Ygps_mod = 0.0;              % GPS position Y accoarding to the model [m]
        Parameters.Hgps_mod = 0.0;              % GPS position height accoarding to the model   [m]
        

        %
        Parameters.uneven_mass = false;    % true = use uneven mass distribution in bike model ; 0 = do not use

    elseif strcmp(bike,'black')
        % Black bike

        % Parameters on bike (actual measured)
        Parameters.inertia_front = 0.245;  %[kg.m^2] inertia of the front wheel
        Parameters.h = 0.534 ;             % height of center of mass [m]
        Parameters.lr = 0.4964;             % distance from rear wheel to frame's center of mass [m]
        Parameters.lf = 1.095-0.4964;       % distance from front wheel to frame's center of mass [m]
        Parameters.c = 0.06;               % length between front wheel contact point and the extention of the fork axis [m]
        Parameters.m = 31.3;               % Bike mass [kg]
        Parameters.lambda = deg2rad(66);   % angle of the fork axis [deg]  !!! TO BE MEASURED
        Parameters.IMU_height = 0.215;     % IMU height [m]
        Parameters.IMU_x = 0.0;           % x Position of the IMU measured from rear wheel (parallel to bike) [m]
        Parameters.IMU_roll = 0;           % Orientation offset in roll (degrees)
        Parameters.IMU_pitch = 0;            % Orientation offset in pitch (degrees)
        Parameters.IMU_yaw = 0;             % Orientation offset in yaw (degrees)
        Parameters.Xgps = 0.0;             % Actual GPS position offset X (measured from middlepoint position parralel to bike heading) [m]
        Parameters.Ygps = 0.0;             % Actual GPS position offset Y (measured from middlepoint position perpendicular to bike heading) [m]
        Parameters.Hgps = 0.0;             % GPS position height (measured from the groud to the GPS)   [m]
        
        % Parameters in model 
        Parameters.inertia_front_mod = 0.245;  %[kg.m^2] inertia of the front wheel
        Parameters.h_mod = 0.534 ;             % height of center of mass [m]
        Parameters.lr_mod = 0.4964;             % distance from rear wheel to frame's center of mass [m]
        Parameters.lf_mod = 1.095-0.4964;       % distance from front wheel to frame's center of mass [m]
        Parameters.c_mod = 0.06;               % length between front wheel contact point and the extention of the fork axis [m]
        Parameters.m_mod = 31.3;               % Bike mass [kg]
        Parameters.lambda_mod = deg2rad(66);   % angle of the fork axis [deg]  !!! TO BE MEASURED
        Parameters.IMU_height_mod = 0.215;     % IMU height [m]
        Parameters.IMU_x_mod = 0.0;            % x Position of the IMU measured from rear wheel (parallel to bike) [m]
        Parameters.IMU_roll_mod = 0;           % Orientation offset in roll (degrees)
        Parameters.IMU_pitch_mod = 0;            % Orientation offset in pitch (degrees)
        Parameters.IMU_yaw_mod = 0;             % Orientation offset in yaw (degrees)
        Parameters.Xgps_mod = 0.0;             % Actual GPS position offset X (measured from middlepoint position parralel to bike heading) [m]
        Parameters.Ygps_mod = 0.0;             % Actual GPS position offset Y (measured from middlepoint position perpendicular to bike heading) [m]
        Parameters.Hgps_mod = 0.0;             % GPS position height (measured from the groud to the GPS)   [m]
       
        %
        Parameters.uneven_mass = false;    % true = use uneven mass distribution in bike model ; 0 = do not use
         
    end
end
