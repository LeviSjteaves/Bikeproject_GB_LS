
%% Generate roll reference
Ts_ref = Ts; % [s] Sampling time
time_balance = 0; % Time for the bike to be balanced
t = 0:Ts_ref:sim_time;
amp_in_deg = 3;% deg
amp = 2*deg2rad(amp_in_deg);
len_for_one_step = 10; % secs
going_straight_time_aft_ctl_start = 7;
period_T = len_for_one_step*2; % secs

y = zeros(size(t,1),size(t,2));
idle_range = going_straight_time_aft_ctl_start/Ts_ref;

% Straight Line Wave
y(idle_range+1:end) = amp.*square(2*pi/period_T*t(1:end-idle_range) + pi/2)/2;


% Going Straight
% y = 0.*t;

% Constant Circle
% y = deg2rad(3).*ones(size(t,1),size(t,2)); 

% Infinity 
% period_T = 360/(amp_in_deg/(v^2/(g*b))) * 2.85 / 4;
% y(idle_range+1:end) = amp.*square(2*pi/period_T*t(1:end-idle_range))/2;


% figure
% plot(t,rad2deg(y))
% phi_ref = y;
% title('Roll references generated')
% ylabel('Roll / degree')
% xlabel('Time / sec')
%% Export to CSV

% csv_name = ['inf_v' num2str(v) '_d' num2str(amp_in_deg) '.csv'];
% writematrix([t.',-y.'],csv_name);
