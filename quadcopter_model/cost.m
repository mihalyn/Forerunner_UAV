function J = cost(obs,cctr_chain,Q,R,S, d_r, d_x_k, du)
%MPC cost for one step

% Estimated future outputs of plant
%1st:  1: 6   i*6-5 : i*6 
%2nd:  7:12  (1)-hez: 6*i+1 6*(i+1)
%3rd: 13:18
%4th: 19:24
%5th: 25:30


% obs=C*A;
% ctr=zeros(6,4);
% cctr=zeros(6,4);
% for i=1:h-1 % fill powers row by row
%     obs(6*i+1:6*(i+1),:)=obs(6*i-5:6*i,:)*A;
%     ctr(6*i+1:6*(i+1),:)=A*ctr(6*i-5:6*i,:)+B;
%     cctr(6*i+1:6*(i+1),:)=C*A*ctr(6*i-5:6*i,:)+C*B;
% end
% 
% 
% cctr_chain=zeros((h)*length(A),(h-1)*size(B,2));
% % ez h-szor egymas melle pakolva a cctr, also 3szog alakban
% 
% for i=0:h-2 % fill up: copying one column next to each other and sliding down with one block matrix's rows
%     cctr_chain(6*i+1:end,4*i+1:4*(i+1))=cctr(1:end-6*i,:); %chain: 4i-3:4i
% end

% for i=2:h
%     obs(i,:)=obs(i-1,:)*A;
%     ctr(i,:)=A*ctr(i-1,:);
% end    
% cctr=C.*ctr;   

error=d_r(:) - obs*d_x_k+cctr_chain*du;

%% Future errors:
% d_y_k_1=   C_A*d_x_k;
% d_y_k_2= C_A_2*d_x_k +           C_B*du1;
% d_y_k_3= C_A_3*d_x_k + (C_A_B + C_B)*du1 + C_B*du2;
% d_y_k_4= C_A_4*d_x_k + (C_A_2*B + C_A_B + C_B)*du1 + (C_A_B+C_B)*du2+C_B*du3;

% control effort rate
ddu=du(5:end)-du(1:end-4);

J = error'*Q*error + du'*R*du + ddu'*S*ddu;
end
%%
% subs(J,['x_k' 'y_k'],[zeros([6 1]) zeros([6 1])]);
% subs(J,['u_k'],[zeros([4 1])]);
