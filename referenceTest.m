function [initialization_new] = referenceTest(traj,Th,Ts,initialization)
    % Check if the trajectory is valid
    traj_new = traj;

    % Minimun number of reference points
    if N<(Th/Ts)
        disp('Not enough reference points');

    end
    
    % Initialization errors
    if traj(1,1) ~= initialization(1,1) || traj(1,2) ~= initialization(1,2) || traj(1,3) ~= initialization(1,3)
        disp('initialization is not correct');
        initialization_new(1,1) = traj(1,1);
        initialization_new(1,2) = traj(1,2);
        initialization_new(1,3) = traj(1,3);
    end
        
    % Density of datapoints

Min_density_distance = 1;

traj(n,1)  
if dist > Min_density_distance
    
end    
        
end
