function [initialization_new] = referenceTest(traj,Th,Ts,initialization,v)
    % Check if the trajectory is valid
    [n] = size(traj);

    %% Minimun number of reference points
    if n<(Th/Ts)
        disp('Warning: Not enough reference points');
    end
    
    %% Initialization errors
    initialization_new = initialization;
    if traj(1,1) ~= initialization(1) || traj(1,2) ~= initialization(2) || traj(1,3) ~= initialization(3)
        disp('Warning: initialization has been corrected');
        initialization_new(1) = traj(1,1);
        initialization_new(2) = traj(1,2);
        initialization_new(3) = traj(1,3);
    end
        
    %% Density of datapoints
    Min_density_distance = 10;
    index = 0;
    for i = 1:n-1
        dist = sqrt((traj(i,1)-traj(i+1,1))^2+(traj(i,2)-traj(i+1,2))^2);
        if dist > Min_density_distance
            index = [index i];
        end
    end    
    if length(index) ~= 1
        disp('Warning: Some ref points are more than 10m apart from each other, index: ')
        disp(index(2:end))
    end
  
    %% Warns for sharp turns which are hard to reach for the bike 
    index = 0;
    for i = 1:n-1
       if abs(traj(i,3)-traj(i+1,3))> pi/3 
           index = [index i];
       end
    end
    if length(index) ~= 1
        disp('Warning: Sharp turn during the trajectory, index: ')
        disp(index(2:end))
    end

    
%it warns when a turn of >90 will be made within ts/v
%Determine amount of datapoints which should be added
%Include dpsiref in this matlab script

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% turn_distance = 0;
% max_turn = pi/6;
% index = 0;
% 
% for i = 1:1:length(Results.dpsiref.Data)-turn_distance
% tot_dpsiref = sum(Results.dpsiref.Data(i:i+turn_distance));
% if tot_dpsiref >= max_turn   
%     index = [index i];
% end
% end
% if length(index) ~= 1
% disp('Message: An unrealistic sharp turn has been detected, index: ');
% disp(index(2:end))
% end


    %% Check if bike travels much longer distance than trajectory point distance
    bike_dis=v*Ts;
    tra_dis=0;
    for i = 1:n-1
       tra_dis = sqrt((traj(i,1)-traj(i+1,1))^2+(traj(i,2)-traj(i+1,2))^2);
    end
 
    if bike_dis>tra_dis %if bike_dis is larger a lot than tra_dis, it's difficult to track reference trajectory, display warning
        disp('Warning: Sampling time of the simulation is larger than trajectory sampling');
    end

end
