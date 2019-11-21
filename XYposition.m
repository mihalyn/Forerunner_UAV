function position=XYposition(input)
    absolutepos = input(1);
    phi = input(2);
    xpos = absolutepos*cos(phi);
    ypos = absolutepos*sin(phi);
    position = [xpos; ypos];
end