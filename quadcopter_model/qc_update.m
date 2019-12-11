function out = qc_update(u, traj_res, tmax)
    
    persistent traject_update
    persistent purposed_track
    persistent UGV_pos UAV_pos
    persistent R
    
    if isempty(traject_update)
        traject_update = 0;
        purposed_track = [0;0;10];
    end
    if isempty(UGV_pos)
        UGV_pos = [0; 0; 0];
    end
    if isempty(UAV_pos)
        UAV_pos = [0; 0; 10];
    end
    
    UGV_pos_current = u(2:4);
    %UAV_pos_current = u(10:12);
    t_actual = u(1);
    
    if t_actual==0
        traject_update = 0;
        purposed_track = [0;0;10];
        UGV_pos = [0; 0; 0];
        UAV_pos = [0; 0; 10];
    end
    
    accel_cent = u(10);
    UGV_accel = u(6);
    UGV_yaw = u(7);
    UGV_yaw_dot = u(8);
    
    T_WB = [cos(UGV_yaw) -sin(UGV_yaw);...
                sin(UGV_yaw) cos(UGV_yaw)]; 
    
    ugv_acc_w = T_WB*[UGV_accel; sign(UGV_yaw_dot)*accel_cent];

    % Trajectory reference update at every tmax
    if mod(t_actual, tmax) == 0
        disp('trajectory update');
        disp(t_actual);
        ts = 0.025;
        
        UGV_vel = u(5);

                      
        %UAV_vel = UAV_meas(4:6);
        %UAV_euler = UAV_meas(7:9);
        
        xyz = [UGV_pos_current(1); UGV_pos_current(2); 10];
        if traject_update == 0
            accel_ref = [0.024; 0; 0];
            UGV_accel = 0.024;
        else
            accel_ref = [UGV_accel*cos(UGV_yaw); UGV_accel*sin(UGV_yaw); 0];
        end
        %xyz0 = UAV_pos_current;
        
        for dt = 0:tmax/traj_res:tmax
            xyz = [xyz [UGV_pos_current(1) + dt*(UGV_vel + dt*UGV_accel)*cos(UGV_yaw + dt*UGV_yaw_dot); ...
            UGV_pos_current(2) + dt*(UGV_vel + dt*UGV_accel)*sin(UGV_yaw + dt*UGV_yaw_dot); 10]];
        end
        
        % calculating Radius of the trajectory circle:
%         d = sqrt((xyz(1,5)-xyz(1,2))^2 + (xyz(2,5)-xyz(2,2))^2);
%         if (xyz(1,5)-xyz(1,2)) ~= 0
%             alpha = atan2((xyz(2,5)-xyz(2,2)),(xyz(1,5)-xyz(1,2)));
%         else
%             alpha = 0;
%         end
%         gamma = 2*(alpha-UGV_yaw);
%         R = sqrt(d^2/(2-2*cos(gamma)));
%         disp(['R: ' num2str(R)]);
        
        for dt = 0:tmax/traj_res:tmax
            % Body to world transformation matrix
            T_WB = [cos(UGV_yaw + dt*UGV_yaw_dot) -sin(UGV_yaw + dt*UGV_yaw_dot);...
                sin(UGV_yaw + dt*UGV_yaw_dot) cos(UGV_yaw + dt*UGV_yaw_dot)];          
            
            % Centripetal acceleration
%             if R ~= 0
%                 accel_cent = (UGV_vel + dt*UGV_accel)^2/R;
%             else
%                 accel_cent = 0;
%             end
            disp(['accel_cent = ', num2str(accel_cent)]); 
            %accel_ref = [accel_ref [UGV_accel*cos(UGV_yaw + dt*UGV_yaw_dot); ...
            %    UGV_accel*sin(UGV_yaw + dt*UGV_yaw_dot); 0]];
            
            accel_ref = [accel_ref [T_WB*[UGV_accel; sign(UGV_yaw_dot)*accel_cent]; 0]];
        end
        accel_ref = accel_ref(:,2:end);
        sdd1 = csape(0:tmax/traj_res:tmax, [zeros(3,1) accel_ref zeros(3,1)], 'clamped');
%         figure(2)
%         fnplt(sdd1);
        %xyz = xyz(:,3:5);
        purposed_track = [purposed_track xyz];
       
        %[s, sd, sdd] = qc_trajgen(xyz, zeros(3,1), tmax, ts);
        
        % Plot the purposed track and the current spline for the UAV
%         figure(1);
%         plot(purposed_track(1,:), purposed_track(2,:));
%         hold on
        %fnplt(s);
        %fnplt(s);
        %plot(UAV_pos_current(1), UAV_pos_current(2), 'o');
%         hold off
%         legend('Purposed track', 'track reference spline', 'UAV position');
%         title('Reference track for the UAV');
        
        %assignin('base', 's', s);
        %assignin('base', 'sd', sd);
        assignin('base', 'sdd', sdd1);
        
        if t_actual ~= 0
            traject_update = traject_update + 1;
        end
    end
    
    % Plot the trajectory of the UAV and UGV (slows down simulation)
%     UGV_pos = [UGV_pos UGV_pos_current];
%     UAV_pos = [UAV_pos UAV_pos_current];
%     
%     figure(1);
%     plot(UGV_pos(1,:), UGV_pos(2,:));
%     hold on
%     plot(UAV_pos(1,:), UAV_pos(2,:));
%     hold off
%     legend('UGV position', 'UAV position');
%     title('Position of the vehicles');
    
    
    out = [traject_update; accel_cent; ugv_acc_w];
end


