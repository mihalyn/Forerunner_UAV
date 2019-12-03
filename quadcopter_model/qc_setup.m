function [ s, sd, sdd, P, G, G_roll, G_pitch, G_yaw, S, R, Q ] = qc_setup(xyz, xyz0, tmax, ts, t_mpc, h)
%QC_SETUP gets xyz, xyz0, tmax, ts, tmpc, initialEuler, initialPos
%   Detailed explanation goes here

% initialEuler = deg2rad([0, 0, 0, 0, 0, 0]);
% initialPos = [0, 0, 0, 0, 0, 0];
% 
% tmax = 5;

% 
% % Trajectory building - q(r) function
% 
% xyz0 = [0;0;0];
% resolution=pi/18;
% pts = 0:resolution:6*pi;
% 
% %Helix
% end_height = 0;
% xyz = [sin(pts); cos(pts); linspace(0,end_height,length(pts))];


[s, sd, sdd]    = qc_trajgen(xyz, xyz0, tmax, ts);
[ P ]           = qc_param;
assignin('base', 'P', P);
[ As, Bs ]      = qc_sim_trajlin(P,t_mpc);
[ G ]           = qc_num_trajlin(Bs,s,t_mpc,tmax);  %constant
assignin('base', 'G', G);
% Controller tuning for LPV inner loop
[ G_roll,...
  G_pitch,...
  G_yaw ]       = qc_pid_attitude(ts);
assignin('base', 'G_roll', G_roll);
assignin('base', 'G_pitch', G_pitch);
assignin('base', 'G_yaw', G_yaw);
[ S, R, Q ]     = qc_mpcweight(h);  %constant
assignin('base', 'S', S);
assignin('base', 'R', R);
assignin('base', 'Q', Q);
end

