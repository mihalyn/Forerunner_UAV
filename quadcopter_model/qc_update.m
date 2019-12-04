function out = qc_update(u, traj_res, tmax)
    
    persistent traject_update
    persistent purposed_track
    
    if isempty(traject_update)
        traject_update = 0;
        purposed_track = [0;0;0;];
    end
    
    t_actual = u(1);
    disp(t_actual);
    if mod(t_actual, tmax) == 0
        disp('trajectory update');
        disp(t_actual);
        ts = 0.025;
        UGV_pos = u(3:5);
        UGV_vel = u(2);
        UGV_yaw = u(6);
        UGV_yaw_dot = u(7);
        
        UAV_pos = u(9:11);
        %UAV_vel = UAV_meas(4:6);
        %UAV_euler = UAV_meas(7:9);
        
        xyz = [UGV_pos(1); UGV_pos(2); 100];
        xyz0 = UAV_pos;
        
        for dt = 0:tmax/traj_res:tmax
            xyz = [xyz [UGV_pos(1) + dt*UGV_vel*cos(UGV_yaw + dt*UGV_yaw_dot); ...
                UGV_pos(2) + dt*UGV_vel*sin(UGV_yaw + dt*UGV_yaw_dot); 100]];
        end
        purposed_track = [purposed_track xyz];
       
        [s, sd, sdd] = qc_trajgen(xyz, xyz(:,1), tmax, ts);
        figure(1);
        plot(purposed_track(1,:), purposed_track(2,:));
        hold on
        fnplt(s);
        hold off
        assignin('base', 's', s);
        assignin('base', 'sd', sd);
        assignin('base', 'sdd', sdd);
        
        if t_actual ~= 0
            traject_update = traject_update + 1;
        end
    end
    out = traject_update;
end


