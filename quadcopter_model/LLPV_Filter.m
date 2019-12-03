% Do not run separately. Drone dynamics.
% Nonlinear filter for the decoupled LPV inner loop

function out = LLPV_Filter(in, phi0, theta0, P)
    T = [P.Jx,0,-P.Jx*sin(theta0);
    0,P.Jy*cos(phi0),P.Jy*cos(theta0)*sin(phi0);
    0,-P.Jz*sin(phi0),P.Jz*cos(phi0)*cos(theta0)];

    out = T*in;
end