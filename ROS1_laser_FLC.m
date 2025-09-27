rosinit('http://localhost:11311');
% rosinit('localhost', 11311);
% Create publisher for velocity commands (cmd_vel)
velPub = rospublisher('/RosAria/cmd_vel', 'geometry_msgs/Twist');

% % Create subscribers for laser scan data and odometry
laserSub = rossubscriber( "/RosAria/laser", "sensor_msgs/LaserScan");  % Specify the callback function
odomSub = rossubscriber( '/RosAria/pose', 'nav_msgs/Odometry' );    % Specify the callback function
% sim_lms1xx_1_laserscan
% /p3dx_1
% Create a message for velocity commands
velMsg = rosmessage(velPub);
sampleDuration =0.1;
maxIter = 5000;
% goals = [3.5 3];
% goals = [1 1.5;3.5 3];
% goals = [1.3 -0.3; 2 -1]
% goals = [3.5 0.1];
goals = [3.5 -0.85];

% goals = [1.2 -0.2];
% goals = [0.8 0; 1.5 1.5];
% goals = [4 -4];
% goals = [5 0];
% to goal driver
to_goal_driver = readfis('goal_v5');
to_goal_driver = convertToType2(to_goal_driver);
to_sub_goal_driver = readfis('subgoal_v4');
to_sub_goal_driver = convertToType2(to_sub_goal_driver);
obs_avoidance_driver_prior_l = readfis('Laser_obs_av_l');
obs_avoidance_driver_prior_r = readfis('Laser_obs_av_r');

obs_avoidance_driver_prior_l = convertToType2(obs_avoidance_driver_prior_l);
obs_avoidance_driver_prior_r = convertToType2(obs_avoidance_driver_prior_r);

smoother = readfis('smoother_v4');