function [ P ] = qc_param(  )
%QC_PARAM Summary of this function goes here
%   Detailed explanation goes here
%% Parameters
% Gravitational acceleration for Budapest
P.gravity = 9.82923;
   
% Physical parameters of drone
P.mass  = 1.271;            % Mass
P.l     = 0.23;             % Arm length
P.Jx    = 0.028238676;      % Elements of the inertial matrix
P.Jy    = 0.028577526;      % Elements of the inertial matrix
P.Jz    = 0.056135038;      % Elements of the inertial matrix

% Propeller parameters
P.b     = 1.14e-6;          % Drag force coefficient
P.k     = 2.98e-5;          % Thrust force coefficient

% Saturation values - feel free to edit
P.Tmot  = 1;
P.Fsat  = P.k*P.Tmot/P.b;
P.Tmax  = 4*P.Fsat;
P.taux = 4*P.Fsat*P.l/sqrt(2);
P.tauy = 4*P.Fsat*P.l/sqrt(2);
P.tauz = 4*P.Fsat*P.b/P.k;

% Trim value
P.trim  = P.gravity*P.mass/4;

end

