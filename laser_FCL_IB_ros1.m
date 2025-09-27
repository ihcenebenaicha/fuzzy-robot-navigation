clear all; clc;
% close all
run('ROS1_laser_FLC');
% Receive odometcry data to get the robot's pose
odomData = receive(odomSub, 10);  % Wait for 10 seconds to receive data                                                                                 

% Extract the robot's position (x, y, z) and orientation (quaternion)
position = odomData.Pose.Pose.Position;
orientation = odomData.Pose.Pose.Orientation;
% Convert quaternion to Euler angles (roll, pitch, yaw)
angles = quat2eul([orientation.W orientation.X orientation.Y orientation.Z]);   
yaw = angles(1);  % Yaw is the robot's orientation around the z-axis
pause(sampleDuration);
robot.x= position.X; robot.y= position.Y; robot.theta = yaw;
robot.v= 0; robot.w = 0;
epsilon =.2;                                                                    
i=1;
j = 0;   
robot_stuckvelPub=0; 

global_goal_reached = 0;
goal_index = 1;
l=0;
while global_goal_reached == 0
    distToGoal = norm(goals(goal_index,:)-[position.X position.Y]);
    goal_index
    while i< maxIter && distToGoal > epsilon 
        i;
        
        %% sense
        % Receive laser scan data
        scanData = receive(laserSub, 10);  % Wait for 10 seconds to receive data
        % Extract the ranges and angles from the laser scan
        
        [n,m] = size(scanData.Ranges);
        midl_obs = mean(scanData.Ranges(n/2-30:n/2+30));
        [range_scan,index_nearest_obs] = min(scanData.Ranges);
        angles_scan = scanData.AngleMin:scanData.AngleIncrement:scanData.AngleMax;
        angle_obs_robot = angles_scan(index_nearest_obs);
                                                
        % Receive odometry data to get the robot's pose
        odomData = receive(odomSub, 10);  % Wait for 10 seconds to receive data
        % Extract the robot's position (x, y, z) and orientation (quaternion)
        position = odomData.Pose.Pose.Position;
        orientation = odomData.Pose.Pose.Orientation;
        % Convert quaternion to Euler angles (roll, pitch, yaw) 
        angles_rob = quat2eul([orientation.W orientation.X orientation.Y orientation.Z]);
        yaw = angles_rob(1); % Yaw is the robot's orientation around the z-axis
        robot.x= position.X;
        robot.y= position.Y;                                            
        robot.theta = yaw;
                                    
        distToGoal = norm(goals(goal_index,:)-[position.X position.Y]);                                        
        thetad = atan2(goals(goal_index,2)-position.Y, goals(goal_index,1)-position.X);
        thetaGoal= -thetad + yaw;             
        if thetaGoal < -3.14
            thetaGoal = thetaGoal + 6.28;
        elseif thetaGoal > 3.14
            thetaGoal = thetaGoal - 6.28;
        end
        if range_scan < 2 &&  range_scan > 0.2                           
            Kw = evalfis(smoother,[1.2*range_scan, angle_obs_robot]); 
        elseif range_scan < 0.2 
            Kw = 1;                                         
        else f
            Kw = 0;
        end 

%         if range_scan> 0.4
%             Kw = 0;
%         else
%             Kw = 1;
%         end

        left_close = find_vid_v2_ros1(scanData, robot, goals, goal_index, "l", 0.4);
        right_close = find_vid_v2_ros1(scanData, robot, goals, goal_index, "r", 0.4);
        
        %% Set commands 
        % Set linear and angular velocity
        [numb_goals, a] = size(goals);
        if goal_index < numb_goals
            u1= evalfis(to_sub_goal_driver,[distToGoal, thetaGoal]);
        else
            u1= evalfis(to_goal_driver,[distToGoal, thetaGoal]);
        end
        
        u2 = evalfis(obs_avoidance_driver_prior_l,[range_scan, angle_obs_robot]);
        if right_close.dist_to_g < left_close.dist_to_g            
            u2 = evalfis(obs_avoidance_driver_prior_r,[range_scan, angle_obs_robot]);
        else                                    
            u2 = evalfis(obs_avoidance_driver_prior_l,[range_scan, angle_obs_robot]);                                   
        end
        
        no_vide_l = no_vide_ros1(scanData,"l");                     
        no_vide_r = no_vide_ros1(scanData,"r");
        
        [goalIsolated, not_on_my_ranges] = goal_isolated_ros1(n, thetaGoal, distToGoal,  scanData);
% index_nearest_obs < n/q'2+55, index_nearest_obs > n/2-55,
%         if all([no_vide_r, not(no_vide_l), goalIsolated,  l==0])
%             edge_index = left_close.edge;
%             edge_x = robot.x + scanData.Ranges(edge_index)*cos(angles_scan(edge_index)+yaw);
%             edge_y = robot.y + scanData.Ranges(edge_index)*sin(angles_scan(edge_index)+yaw);
%             new_sub_goal = [edge_x - 0.3, edge_y - 0.3]
%             goals = [goals(1:end-1,:); new_sub_goal; goals(end,:)]
%             l = 1;
%         elseif all([no_vide_l, not(no_vide_r), goalIsolated, l==0])
%             edge_index = right_close.edge;
%             edge_x = robot.x + scanData.Ranges(edge_index)*cos(angles_scan(edge_index)+yaw);
%             edge_y = robot.y + scanData.Ranges(edge_index)*sin(angles_scan(edge_index)+yaw);
%             new_sub_goal = [edge_x - 0.3, edge_y - 0.3]
%             goals = [goals(1:end-1,:); new_sub_goal; goals(end,:)]
%             l = 1;
%         elseif no_vide_l && no_vide_r
%             no_vide_l && no_vide_r
%             while not_on_my_ranges
%                 distToGoal = norm(goals(goal_index,:)-[position.X position.Y]);                                         
%                 thetad = atan2(goals(goal_index,2)-position.Y, goals(goal_index,1)-position.X);
%                 thetaGoal= -thetad + yaw;             
%                 if thetaGoal < -3.14
%                     thetaGoal = thetaGoal + 6.28;
%                 elseif thetaGoal > 3.14
%                     thetaGoal = thetaGoal - 6.28;
%                 end
%                 scanData = receive(laserSub, 10);
%                 [goalIsolated, not_on_my_ranges] = goal_isolated_ros1(n, thetaGoal, distToGoal,  scanData);
%                 velMsg.Angular.Z = -0.1;
%                 velMsg.Linear.X = 0.0;
%                 send(velPub, velMsg);
%             end
%             if not(goalIsolated)
%                 velMsg.Angular.Z = 0.0;
%                 send(velPub, velMsg);
%                 continue;
%             end
%         end

                                
        if distToGoal < 0.3
            velMsg.Angular.Z = 3*double(u1(1));% Move forward
            velMsg.Linear.X =  3*double(u1(2)); % Slight rotation
        else
            velMsg.Angular.Z = 1*double((1-Kw)*u1(1)+ Kw*u2(1));% Move forward
            velMsg.Linear.X =  1*double((1-Kw)*u1(2)+ Kw*u2(2)); % Slight rotation
        end
%
%         u1= evalfis(to_goal_driver,[distToGoal, thetaGoal]);
%         velMsg.Angular.Z = 3*double(u1(1));% Move forward
%         velMsg.Linear.X =  3*double(u1(2)); % Slight rotation
        ww(i)= velMsg.Angular.Z; vv(i)= velMsg.Linear.X;

        send(velPub, velMsg);
%         k(i) = Kw;
%         u(i) = u2(1);
        yy(i) = robot.y; xx(i)=robot.x; tth(i) = yaw;   
        % if i == 1
        %     last_x = 0;
        %     last_y = 0;
        % else
        %     last_x = xx(i-1);
        %     last_y = yy(i-1);
        % end
        % 
        % if norm([robot.x robot.y] - [last_x last_y]) < 0.01 && i~=1
        %     if j == 10 
        %         robot_stuck =1;    
        %     end
        %     j=j+1;
        % else
        %     j = 0;
        % end
        i = i+1;
        
    end
    if goals(goal_index, :) == goals(end,:)
        global_goal_reached = 1;
    end
    goal_index = goal_index+1;

end
t = 1:i-1;
dxdt = gradient(xx, t);                                     
dydt = gradient(yy, t);                                      
Dist = trapz(t, sqrt(dydt.^2 + dxdt.^2))

% Set linear and angular velocity
velMsg.Linear.X = 0.0;  % Move forward 
velMsg.Angular.Z = 0.0; % Slight rotation 
ww(i+1) =0.0;vv(i+1) =0.0;
% Send the velocity command 
send(velPub, velMsg);
pause(0.1);
% Shutdown ROS connection when done
rosshutdown;
%% Display the robot's position and orientation (yaw)
figure;
plot(xx,yy);
hold on
% p = nsidedpoly(1000, 'Center', [-0.2 0.8], 'Radius', 0.19);
% plot(p, 'FaceColor', 'r')

plot(goals(:,1),goals(:,2),'*');
hold off
grid on;
figure; plot(vv);
figure; plot(ww);
% figure; plot(k);