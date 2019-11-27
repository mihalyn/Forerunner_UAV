clear all;

addpath('car_model', 'quadcopter_model');

% Simulation variables
dT = 0.005;         % [s]
duration = 50;      % [s]
time = [0:dT:duration-dT];

% Generate reference signals for the UGV model

UGV_init = [0.3, 0, 0];

Force_ref(1:length(time)/2) = 30;
Force_ref(length(time)/2:length(time)) = 10;
Force_in = timeseries(Force_ref, time);

Steering_ref(1:length(time)/4) = pi/3;
Steering_ref(length(time)/4:2*length(time)/4) = -pi/4;
Steering_ref(2*length(time)/4:length(time)) = pi/6;
Steering_in = timeseries(Steering_ref, time);

% Initialize UAV model

time_for_traj = 1; % [s]
traj_res = 20;
xyz = [linspace(0,cos(UGV_init(2))*UGV_init(1)*time_for_traj,traj_res); ...
    linspace(0,sin(UGV_init(2))*UGV_init(1)*time_for_traj,traj_res); 20*ones(1,traj_res)];
xyz0 = [0;0;0];
%plot3(xyz(1,:), xyz(2,:), xyz(3,:));

initialEuler = deg2rad([0, 0, UGV_init(2), 0, 0, 0]);
initialPos = [0, 0, 0, UGV_init(1), 0, 0];

qc_setup(xyz, xyz0, time_for_traj);


