function [ As, Bs ] = qc_sim_trajlin(P,t_mpc)
%QC_SIM_TRAJLIN setups the model for MPC control


%% Building symbolic SS matrices
syms px pz vx vy vz phi theta f
py = sym('py');
psi = sym('psi');

% Position change
pxdot = vx;
pydot = vy;
pzdot = vz;
    
% Rotational matrix
R_roll = [1, 0, 0;
		0, cos(phi), sin(phi);
		0, -sin(phi), cos(phi)];
R_pitch = [cos(theta), 0, -sin(theta);
		0, 1, 0;
		sin(theta), 0, cos(theta)];
R_yaw = [cos(psi), sin(psi), 0;
		-sin(psi), cos(psi), 0;
		0, 0, 1];

R = R_roll*R_pitch*R_yaw;  
    
% Sum of Forces
vgdot = R.'*[0;0;f/P.mass]-[0;0;P.gravity];
vxdot = vgdot(1);
vydot = vgdot(2);
vzdot = vgdot(3);
    
xyz = [pxdot; pydot; pzdot];
vxyz = [vxdot; vydot; vzdot];

if xyz ~= 0
xyz = simplify(xyz,'Steps',50);
end
if vxyz ~= 0
vxyz = simplify(vxyz,'Steps',50);
end

syms x0 x1 x2 x3 y0 y1 y2 y3 z0 z1 z2 z3 t

% States and inputs

x = [px;py;pz;vx;vy;vz];
u = [f;phi;theta;psi];
xdot = [xyz; vxyz];

% Quadratic spline formula

q0 = [x0;y0;z0];
q1 = [x1;y1;z1];
q2 = [x2;y2;z2];
q3 = [x3;y3;z3];
q = q0 + q1*t + q2*t^2 + q3*t^3;

% Inversion of linearized equations

xt = [q(1);q(2);q(3);diff(q(1),t);diff(q(2),t);diff(q(3),t)];
ft = (diff(q(3),t,2)+P.gravity)*P.mass;
thetat = atan(diff(q(1),t,2)/(diff(q(3),t,2)+P.gravity));
phit = atan(-cos(thetat)*diff(q(2),t,2)/(diff(q(3),t,2)+P.gravity));
psit = 0;
ut = [ft;phit;thetat;psit];

% Discretization

xdot = xdot*t_mpc+x; %FIXED

% SS matrices linearized around spline trajectory

As = [diff(xdot,x(1)) diff(xdot,x(2)) diff(xdot,x(3)) diff(xdot,x(4)) diff(xdot,x(5)) diff(xdot,x(6))];
%As = subs(As, [x u], [xt ut]);
Bs = [diff(xdot,u(1)) diff(xdot,u(2)) diff(xdot,u(3)) diff(xdot,u(4))];
Bs = subs(Bs, [x u], [xt ut]);

end

