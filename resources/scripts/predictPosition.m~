function [X, Y, theta] = predictPosition(vec_Control, numSteps)
    %%%% This function takes in the vector of controls
    %%%% (ex: [0.0, 0.0, 0.2, 0.3, -0.4, -0.2])  
    %%%% and the steps taken per control instruction (ex: 3).
    %%%% Note that it returns the global pose from origin, defined as X=0,
    %%%% Y=0, and theta = 0.
    
    % Initialize dictionary to hold model, initialize return vectors
%     vec_Control = num2str(vec_Control);
    disp(vec_Control)
    conVect = [0, 0.0000, 1.3408;0.1, 0.3037, 0.9993;0.2, 0.3053, 0.9891;0.3, 0.3136, 0.9589;0.4, 0.3302, 0.8479;0.5, 0.2889, 0.6879;0.6, 0.2049, 0.6274;0.7, 0.1965, 0.6429;0.8, 0.151, 0.6724;0.9, 0.0731, 0.6416;-0.1, -0.3037, 0.9993;-0.2, -0.3053, 0.9891;-0.3, -0.3136, 0.9589;-0.4, -0.3302, 0.8479;-0.5, -0.2889, 0.6879;-0.6, -0.2049, 0.6274;-0.7, -0.1965, 0.6429;-0.8, -0.151, 0.6724;-0.9, -0.0731, 0.6416];
    controlModel = containers.Map;
    X = zeros(length(vec_Control),1);
    Y = zeros(length(vec_Control),1);
    theta = zeros(length(vec_Control),1);
    X(1) = 0.0; Y(1) = 0.0; theta(1) = 0.0;
    
    % Create the dictionary
    for i = 1:19
        controlModel(num2str(conVect(i,1))) = [conVect(i,2), conVect(i,3)];
    end
    
    frequency = 4.5;
    time_step = numSteps*(1/frequency);
    for j = 2:length(vec_Control)+1
        indx = num2str(vec_Control(j-1));
        temp = controlModel(indx);
        disp(temp)
        w = temp(1); v = temp(2);
        theta(j) = theta(j-1) + time_step*w;
        robot_vx = v;
        robot_vy = 0;
        v_x = (robot_vx * cos(theta(j))) - (robot_vy * sin(theta(j)));
        v_y = (robot_vx * sin(theta(j))) + (robot_vy * cos(theta(j)));

        X(j) = X(j-1) + (v_x)*time_step;
        Y(j) = Y(j-1) + (v_y)*time_step;
        
    end
    

end