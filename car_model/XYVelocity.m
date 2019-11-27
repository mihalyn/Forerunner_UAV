function velocity=XYVelocity(input)
    absolutevel = input(1);
    phi = input(2);
    xvel = absolutevel*cos(phi);
    yvel = absolutevel*sin(phi);
    velocity = [xvel; yvel];
end