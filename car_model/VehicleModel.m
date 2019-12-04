function output = VehicleModel(u, ts_car, UGV_init)

    persistent v_k phi_dot_k beta_k step

    if isempty(v_k)
        v_k = UGV_init(1);
        step = 1;
    end
    if isempty(phi_dot_k)
        phi_dot_k = UGV_init(2);
    end
    if isempty(beta_k)
        beta_k = UGV_init(3);
    end
    disp([num2str(step) ' ' num2str(u(1)) ' ' num2str(u(2))]);
    % input(1): v
    % input(2): phi_dot legyezési szögsebesség
    % input(3): beta sideslip
    % input(4): longitudinal force
    % input(5): steering angle/kormányszög

    % output(1): longitudinal acceleration
    % output(2): phi_dotdot legyezési szöggyorsulás
    % output(3): beta_dot

    %Longitudinal Vehicle Model

    %Constants
    m=2023; %Mass of the vehicle
    A=2;    %Surface of the Vehicle
    C=.7;   %Drag coefficient
    ro=1.2; %Density of the air
    
    %Equations
    v_next =((-1/2*(v_k^2*A*ro*C)+u(1))/m)*ts_car + v_k;

    % Lateral Vehicle Model

    %Constants
    m=2023;     %Mass of the vehicle
    J=6286;     %Yaw-inertia
    C1=180000;  %Front cornering coefficient
    C2=230000;  %Rear cornering coefficient
    l1=1.265;   %Distance1
    l2=1.9;     %Distance2

    %Equations
    phi_dot_next = (phi_dot_k*((-1*C1*l1^2 - C2*l2^2)/abs(v_k)/J) + beta_k*((-1*C1*l1 + C2*l2)/J) + u(2)*C1*l1/J)*ts_car + phi_dot_k;
    beta_next = (phi_dot_k*((-1*C1*l1 + C2*l2)/m/abs(v_k)^2 - 1) + beta_k*((-1*C1-C2)/m/abs(v_k)) + u(2)*C1/m/abs(v_k))*ts_car + beta_k;
    
    output = [v_next phi_dot_next beta_next];
    disp(output);
    v_k = v_next;
    phi_dot_k = phi_dot_next;
    beta_k = beta_next;
    step = step + 1;
end