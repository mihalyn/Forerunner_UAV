function qc_out = QC_mpc_rule(u, sdd,  P, tmax)
%% Nominal control supplemented with MPC error regulation
% 
%   - Nominal control is computed from trajectory spline and hence independent
%     from A,B,C matrices.
%       + u contains the current time (when acceleration is used)
%       + s, sd, sdd is the trajecotry and its derivatives (acceleration
%       used for inverted equations)
%       + P contains physical parameters 
%
%   - MPC correction variable is computed based on
%       + error dynamics (A,B,C)
%       + h horizon (steps) with t_mpc sampling time
%       + u(2:7) are the current outputs (to produce error signal)
%
                %e = [px;py;pz;vx;vy;vz];        % Error vector
                %r = -K(:,:,floor(t/ts)+1)*e;    % LQ adjustment vector


t0=u(1) - tmax*u(8);
a = fnval(sdd, t0);

%disp(a);
% Inverted linearized equations

z = (a(3)+P.gravity)*P.mass; % sum lift forces = m*a
theta = atan(a(1)/(a(3)+P.gravity));
phi = atan(-cos(theta)*a(2)/(a(3)+P.gravity));
psi = 0;

qc_out = [z phi theta psi a'];
end