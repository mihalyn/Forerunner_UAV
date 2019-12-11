function output  = accel_cent(u)
    v = u(1);
    phi_last = u(2);
    pos = u(3:4);
    pos_last = u(5:6);
    
    % calculating Radius of the trajectory circle:
    d = sqrt((pos(2)-pos_last(2))^2 + (pos(1)-pos_last(1))^2);
    if (pos(1)-pos_last(1)) ~= 0
        alpha = atan2((pos(2)-pos_last(2)),(pos(1)-pos_last(1)));
    else
        alpha = 0;
    end
    
    gamma = 2*(alpha-phi_last);

    if cos(gamma) == 1
        R = 1e5;
    else
        R = sqrt(d^2/(2-2*cos(gamma)));
    end
        
    % The centripetal acceleration:
    a_cp = v^2/R;
    disp(['a_cp: ' num2str(a_cp)]);
    disp(['R: ' num2str(R)]);
    
    output = [a_cp R];
end

