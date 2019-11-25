clear all;

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

% Reference signals for the UAV model

