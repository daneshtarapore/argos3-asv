function [fg] = DrawTrajectory(MRB, MA, DeltaT, m,   Xu1, Yv1, Yr1, Nr1,      Xu2, Yv2, Nr2,    Xu3, Yv3, Nr3,   T, FL, FR, alpha)
    

u = 0; v = 0; % current velocity m/s
r = 0;        % current angular velocity rad./s    
velocity = [u v r]';
x = 0; y = 0; % current position in fixed inertial frame m
psi = 0; % current roll angled in fixed inertial frame radians
position = [x y psi]';


fg  = figure;
for time = 1:100
        
    CRB = [0 -m*r 0; m*r 0 0; 0 0 0];
    CA = [0 0 -(Yv1*v + Yr1*r); 0 0 +Xu1*u; (+Yv1*v+Yr1*r) -Xu1*u 0];  % WE CHECKED THIS EXPRESSION FOR CA -- THOR HAD ALL THE ADDED MASS COEFFICIENTS AS -VE WHICH IS WHY OUR ADDED CORIOLIS COEFFICIENTS CA HAD THE OPPOSITE SIGN
    DL = [Xu2 0 0; 0 Yv2 0; 0 0 Nr2];
    DQ = [Xu3*(abs(u)) 0 0; 0 Yv3*(abs(v)) 0; 0 0 Nr3*(abs(r))];
    
    
    M = MA + MRB;
    acceleration = inv(M)*(T - CRB*velocity - CA*velocity - DL*velocity - DQ*velocity);
    
    
    velocity = velocity + DeltaT * acceleration;
    jacobian = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    
    position = position + DeltaT * (jacobian*velocity); % make sure to keep the roll angle in range [-pi  to pi] or [0 to 2pi]?
    
    
    x = position(1); y = position(2); psi = position(3);
    u = velocity(1); v = velocity(2); r   = velocity(3);
    
    %logposition(time) = position(3); 
    %[time velocity' position']
    figure(fg); hold on; quiver(x,y,0.1*cos(psi),0.1*sin(psi))
    
end

box on
xlabel('X-coordinate (meters)')
ylabel('Y-coordinate (meters)')
title( sprintf('%s%0.1f%s%0.1f%s%0.1f','Left propeller: ', FL, 'N. Right propeller: ', FR, 'N. Propeller angle (degrees): ', alpha*180/pi));
set(gca, axis([0 1 -.1 .1]))