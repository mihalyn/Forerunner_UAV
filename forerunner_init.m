clear all;

addpath('car_model_cont', 'car_model_discr', 'quadcopter_model');

% Simulation variables
ts_car = 0.005;         % [s]
duration = 50;      % [s]
time = 0:ts_car:duration-ts_car;
tmax = 0.5;

% Generate reference signals for the UGV model

transition = 1/20;

UGV_init = [1, 0, 0];

Force_ref(1:length(time)/2) = 50;
Force_ref(length(time)/2:length(time)) = 50;
Force_in = timeseries(Force_ref, time);

Steering_ref = zeros(length(time), 1);
Steering_ref(1:length(time)/4) = 0;
Steering_ref(length(time)/4:(length(time)*(1/4 + transition))) = linspace(0,pi/12, duration*transition/ts_car+1); % transition
Steering_ref(length(time)/4:2*length(time)/4) = pi/12;
Steering_ref(2*length(time)/4:3*length(time)/4) = pi/12;
Steering_ref(3*length(time)/4:length(time)) = pi/12;
Steering_in = timeseries(Steering_ref, time);

% Initialize UAV model

traj_res = 5;
xyz = [linspace(0,cos(UGV_init(2))*UGV_init(1)*tmax,traj_res); ...
    linspace(0,sin(UGV_init(2))*UGV_init(1)*tmax,traj_res); 10*ones(1,traj_res)];
xyz0 = [0;0;10];
%plot3(xyz(1,:), xyz(2,:), xyz(3,:));
        
ts = 0.005;
t_mpc=0.05;
h=4;
%tmax = 0.5;   % time for trajectory

initialEuler = deg2rad([0, 0, 0, 0, 0, 0]);
%initialPos = [0, 0, 10, UGV_init(1)*cos(UGV_init(2)), UGV_init(1)*sin(UGV_init(2)), 0];
initialPos = [0, 0, 10, 1, 0, 0];


qc_setup(xyz, xyz0, tmax, ts, t_mpc, h);


