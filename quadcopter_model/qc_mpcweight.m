function [ S, R, Q ] = qc_mpcweight(h)
%QC_MPCTUNE Summary of this function goes here
%   Detailed explanation goes here


%% Cost function matrix, Inputs (T, phi, theta, psi)
% 0.5m hiba eseten a beavatkozojel korrekcio kb:
mu1 = 1;
mu2 = 1;
mu3 = 1;
mu4 = 1;
Rc = diag([mu1 mu2 mu3 mu4]).*0; 

% Control effort rate
mur1 = 1;
mur2 = 1;
mur3 = 1;
mur4 = 1;
Sc = diag([mur1 mur2 mur3 mur4]).*6;

% Cost function matrix, States (px, py, pz, vx, vy, vz)
%Q_scale=diag([max_pos max_vel]).^0.5;
%scaling is an issue! diff is usual 0.3m
xi1 = 1;
xi2 = 1;
xi3 = 1;
xi4 = 0.1;
xi5 = 0.1;
xi6 = 0.1;
Qc = diag([xi1 xi2 xi3 xi4 xi5 xi6]); % 

% Cost matrices on horizon:
S=kron(eye(h-2),Sc);
R=kron(eye(h-1),Rc);
Q=kron(eye(h),Qc);
clear QC_mpc_rule

end

