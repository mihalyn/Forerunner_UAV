function output=VehicleModel(input)


%Longitudinal Vehicle Model

    %Constants
    m=2023; %Mass of the vehicle
    A=2;    %Surface of the Vehicle
    C=.7;   %Drag coefficient
    ro=1.2; %Density of the air

    %Variables
    v=input(1); %Velocity of the vehicle
    

    %Equations
    a=(-1/2*(v^2*A*ro*C)+input(4))/m;
    out1=a;

%Lateral Vehicle Model

    %Constants
    m=2023;     %Mass of the vehicle
    J=6286;     %Yaw-inertia
    C1=180000;  %Front cornering coefficient
    C2=230000;  %Rear cornering coefficient
    l1=1.265;   %Distance1
    l2=1.9;     %Distance2

    %Variables
    v=input(1);

    %Equations
    A=[(-1*C1*l1^2-C2*l2^2)/abs(v)/J (-1*C1*l1+C2*l2)/J;
        (-1*C1*l1+C2*l2)/m/abs(v)/abs(v)-1 (-1*C1-C2)/m/abs(v)];

    B=[C1*l1/J;
        C1/m/abs(v)];
    
    C=[1,0];
    
   

    out2=A*input(2:3)+B*(input(5)*v/abs(v));


%Output of the function
    output=[out1;out2];



end