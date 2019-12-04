clear all;

addpath('car_model', 'quadcopter_model');

% Simulation variables
ts_car = 0.005;         % [s]
duration = 50;      % [s]
time = 0:ts_car:duration-ts_car;
tmax = 0.5;

% Generate reference signals for the UGV model

UGV_init = [1, 0, 0];

Force_ref(1:length(time)/2) = 50;
Force_ref(length(time)/2:length(time)) = 20;
Force_in = timeseries(Force_ref, time);

Steering_ref(1:length(time)/4) = pi/3;
Steering_ref(length(time)/4:2*length(time)/4) = -pi/4;
Steering_ref(2*length(time)/4:3*length(time)/4) = pi/6;
Steering_ref(3*length(time)/4:length(time)) = -pi/8;
Steering_in = timeseries(Steering_ref, time);

% Initialize UAV model

traj_res = 20;
xyz = [linspace(0,cos(UGV_init(2))*UGV_init(1)*tmax,traj_res); ...
    linspace(0,sin(UGV_init(2))*UGV_init(1)*tmax,traj_res); 20*ones(1,traj_res)];
xyz0 = [0;0;0];
%plot3(xyz(1,:), xyz(2,:), xyz(3,:));
ts = 0.025;
t_mpc=0.05;
h=4;
tmax = 0.5;   % time for trajectory

initialEuler = deg2rad([0, 0, UGV_init(2), 0, 0, 0]);
initialPos = [0, 0, 0, UGV_init(1), 0, 0];

qc_setup(xyz, xyz0, tmax, ts, t_mpc, h);


