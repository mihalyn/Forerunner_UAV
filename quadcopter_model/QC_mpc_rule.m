function qc_out = QC_mpc_rule(u, s, sd, sdd, P, h, t_mpc, G, tmax ,Q,R,S )
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

persistent du_prev plantEstimation
t0=u(1) - tmax*u(8);
k=floor(t0/t_mpc)+1; %FIXME!!!
sldiagviewer.reportInfo(sprintf('Step: %d',k))
%t_mpc=0.2; %FIXME!!
t_end=t0+t_mpc*h;
t=(t0:t_mpc:t_end)';

r = [fnval(s,t);fnval(sd,t)];
a = fnval(sdd,t0);
y=u(2:7); %gyorsulassal ayert nem lenne nehey kiboviteni.....

d_r=diff(r,1,2); %along columns
% if t0<h*ts
%     d_r=d_r*0;
% end
d_x_k=r(:,1)-y;

                %e = [px;py;pz;vx;vy;vz];        % Error vector
                %r = -K(:,:,floor(t/ts)+1)*e;    % LQ adjustment vector

% Inverted linearized equations

z = (a(3)+P.gravity)*P.mass; % sum lift forces = m*a
theta = atan(a(1)/(a(3)+P.gravity));
phi = atan(-cos(theta)*a(2)/(a(3)+P.gravity));
psi = 0;

%% MPC
if 0==t0 %isempty(du_prev) %0==t0
   du_prev=zeros((h-1)*4,1);
   plantEstimation=timeseries('qc_error_sim');%zeros(h,size(G,1),size(G,3));
   %with commenting this out the estimation of previous run can be seen
end
du=du_prev;
cost_k=0;
t_opt=0;

is_on=1; %MPC RUNNING

%%matrixok a controlhoz
A=G(:,:,k).A;
B=G(:,:,k).B;
C=G(:,:,k).C;

x_dim=size(A,1);
u_dim=size(B,2);
%y_dim=size(G(:,:,:,1),1);

if(rem(t0,t_mpc)==0 && is_on)
    Ac = [];
    bc = [];
    Aceq = [];
    bceq = [];
    
    obs=C*A;
    ctr=zeros(6,4);
    cctr=zeros(6,4);
    
    %ctr=zeros(x_dim,du_dim);
    %cctr=zeros(x_dim,du_dim);

    
    for i=1:h-1 % fill powers row by row
        obs(6*i+1:6*(i+1),:)=obs(6*i-5:6*i,:)*A;
        ctr(6*i+1:6*(i+1),:)=A*ctr(6*i-5:6*i,:)+B;
        cctr(6*i+1:6*(i+1),:)=C*A*ctr(6*i-5:6*i,:)+C*B;
    end


    cctr_chain=zeros((h)*length(A),(h-1)*size(B,2));
    % ez h-szor egymas melle pakolva a cctr, also 3szog alakban

    for i=0:h-2 % fill up: copying one column next to each other and sliding down with one block matrix's rows
        cctr_chain(6*i+1:end,4*i+1:4*(i+1))=cctr(1:end-6*i,:); %chain: 4i-3:4i
    end

    %max lehetseges elteres a linearizalttol>> NEM rate!!
    deglim=30;
    radlim=deg2rad(deglim);
    lb=repmat([-z -radlim-phi -radlim-theta -radlim-psi]',h-1,1);
    ub=repmat([ 30-z  radlim-phi  radlim-theta  radlim-psi]',h-1,1);

    %du=fminunc(@(du)cost(A,B,C,R,Q,d_r,d_x_k,du),du_prev);
    tic;
    [du,cost_k]=fmincon(@(du)cost(obs,cctr_chain,Q,R,S,d_r,d_x_k,du),du_prev,Ac,bc,Aceq,bceq,lb,ub);
    %[du,cost_k]=fminunc(@(du)cost(A,B,C,h,R,Q,d_r,d_x_k,du),du_prev);
    t_opt=toc;
    assignin('base','du',du)
    
    [d_y_sim,t_sim,x_sim] = lsim(G(:,:,k),...
        [du_prev(1:4) reshape(du,[4 h-1])],...
        t(1:end-1),d_x_k);
    du_prev=du;
    % correction: to see not the simulated error but the simulated pos on plot
    %d_y_sim becsult hiba *referenciajeltol valo elteres)
    y_sim=r(:,1:end-1)-d_y_sim';
    plantEstimation(floor(t0/t_mpc)+1)=timeseries(y_sim',t(1:end-1));
    assignin('base','plantEstimation',plantEstimation)    
end
qc_out = [z phi theta psi cost_k t_opt du(1:u_dim)']+[ du(1:u_dim)' zeros(1,x_dim)]; %zeros(1,u_dim+x_dim)];

%out = [z phi theta psi cost_k t_opt du(1:du_dim)']+[du(1:u_dim)' zeros(1,x_dim)];

    % K_mpc=double(U_B(1:4,:));
    %du = K_mpc*e;
if(t0>(tmax-t_mpc*h))
    assignin('base','du',du)
    assignin('base','plantEstimation',plantEstimation)
end