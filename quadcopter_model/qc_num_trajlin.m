function [ G ] = qc_num_trajlin(Bs,s,t_mpc,tmax)
%QC_TRAJLIN setups the model for MPC control
syms x0 x1 x2 x3 y0 y1 y2 y3 z0 z1 z2 z3 t

B = zeros(6,4,tmax/t_mpc+1);

% Comment out for parameter tuning
step = t_mpc/s.breaks(2);
for i = 1:(tmax/t_mpc)
    T = (i-1)*t_mpc;
	qc = [  s.coefs(3*floor((i-1)*step)+1,:);
            s.coefs(3*floor((i-1)*step)+2,:);
            s.coefs(3*floor((i-1)*step)+3,:)];

	q0 = qc(:,4);
	q1 = qc(:,3);
	q2 = qc(:,2);
	q3 = qc(:,1);
	
	% A(:,:,i) = subs(As, [x0 x1 x2 x3 y0 y1 y2 y3 z0 z1 z2 z3 t], [q0(1) q1(1) q2(1) q3(1) q0(2) q1(2) q2(2) q3(2) q0(3) q1(3) q2(3) q3(3) T]);
	% A is constant
    B(:,:,i) = subs(Bs, [x0 x1 x2 x3 y0 y1 y2 y3 z0 z1 z2 z3 t],...
        [q0(1) q1(1) q2(1) q3(1) q0(2) q1(2) q2(2) q3(2) q0(3) q1(3) q2(3) q3(3) T]);
    save('matrix.mat','B'); 
end

A = [eye(3) t_mpc*eye(3); zeros(3) eye(3)];
C = eye(6);

G=ss;
for i=1:size(B,3)
    G(:,:,i)=ss(A, B(:,:,i), C, zeros(6,4),t_mpc,...
        'Name', sprintf('qc_hibadinamika_%d',i),...
        'InputName', {'T','roll','pitch','yaw'},...
        'OutputName', {'px','py','pz','vx','vy','vz'});
end

end

