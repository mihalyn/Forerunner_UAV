function out = qc_update(u, traj_res, tmax)
    
    persistent traject_update
    persistent purposed_track
    persistent UGV_pos UAV_pos
    
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
    
    UGV_pos_current = u(3:5);
    UAV_pos_current = u(9:11);
    t_actual = u(1);
    
    if t_actual==0
        traject_update = 0;
        purposed_track = [0;0;10];
        UGV_pos = [0; 0; 0];
        UAV_pos = [0; 0; 10];
    end
    
    % Trajectory reference update at every tmax
    if mod(t_actual, tmax) == 0
        disp('trajectory update');
        disp(t_actual);
        ts = 0.025;
        
        UGV_vel = u(2);
        UGV_yaw = u(6);
        UGV_yaw_dot = u(7);
                      
        %UAV_vel = UAV_meas(4:6);
        %UAV_euler = UAV_meas(7:9);
        
        xyz = [UGV_pos_current(1); UGV_pos_current(2); 10];
        %xyz0 = UAV_pos_current;
        
        % Points for the spline based on the current state of the UGV
        for dt = 0:tmax/traj_res:tmax-tmax/traj_res
            xyz = [xyz [UGV_pos_current(1) + dt*UGV_vel*cos(UGV_yaw + dt*UGV_yaw_dot); ...
                UGV_pos_current(2) + dt*UGV_vel*sin(UGV_yaw + dt*UGV_yaw_dot); 10]];
        end
%         s1 = csape(0:length(xyz)-1, xyz);
%         sd1 = fnder(s1);
%         sdd1 = fnder(sd1);
%         
%         figure(2);
%         fnplt(sd1);
%         hold on
%         fnplt(sdd1);
%         hold off
        
        %xyz = xyz(:,3:5);
        purposed_track = [purposed_track xyz];
       
        [s, sd, sdd] = qc_trajgen(xyz, zeros(3,1), tmax, ts);
        
        % Plot the purposed track and the current spline for the UAV
        figure(1);
        plot(purposed_track(1,:), purposed_track(2,:));
        hold on
        %fnplt(s);
        fnplt(s);
        %plot(UAV_pos_current(1), UAV_pos_current(2), 'o');
        hold off
        legend('Purposed track', 'track reference spline', 'UAV position');
        title('Reference track for the UAV');
        
        assignin('base', 's', s);
        assignin('base', 'sd', sd);
        assignin('base', 'sdd', sdd);
        
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
    
    
    out = traject_update;
end


