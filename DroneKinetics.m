
%% Mass matrix

u = 0; v = 0; % current velocity m/s
r = 0;        % current angular velocity rad./s    

m  =  1; % mass is 1 kg
Iz =  1; % moment of inertia along z-axis is 1.0 Kg m^2
MRB = [m 0 0; 0 m 0; 0 0 Iz];


%% Added mass matrix
% m_ij mass associated with a force on the body in the ith direction due
% to a unit acceleration in the jth direction
Xu1 = .25; % Kg -- OR DO ADDED MASS COEFFICIENTS HAVE NO UNITS
Yv1 = .25; % Kg
Nr1 = .25; % Kg m^2
Yr1 = .25; % Unit ??? -> Kg m

MA  = [Xu1 0 0; 0 Yv1 Yr1; 0 Yr1 Nr1];


%% Coriolis force for rigid body
% r is the angular velocity along z-axis (yaw). r has unit rad/s
CRB = [0 -m*r 0; m*r 0 0; 0 0 0];


%% Coriolis force for added mass
CA = [0 0 -(Yv1*v + Yr1*r); 0 0 +Xu1*u; (+Yv1*v+Yr1*r) -Xu1*u 0];  % WE CHECKED THIS EXPRESSION FOR CA -- THOR HAD ALL THE ADDED MASS COEFFICIENTS AS -VE WHICH IS WHY OUR ADDED CORIOLIS COEFFICIENTS CA HAD THE OPPOSITE SIGN


%% Linear drag forces
Xu2 = 10; % Kg / s 
Yv2 = 10; % Kg / s
Nr2 = 10; % Kg m^2 / s
DL = [Xu2 0 0; 0 Yv2 0; 0 0 Nr2];


%% Quadractic drag forces -- can be ignored if the boat is moving at low
%% speeds < abs(+/-2m/s)
Xu3 = 0; % Kg / s 
Yv3 = 0; % Kg / s
Nr3 = 0; % Kg m^2 / s
DQ = [Xu3*(abs(u)) 0 0; 0 Yv3*(abs(v)) 0; 0 0 Nr3*(abs(r))];



%% Thrust force -- two propellers with no rotation possible

FL =  1; % Newtons
FR =  1;
DeltaY = 0.2; % distance of propeller to y coordinate of CoM. Unit: m 

% two propellers with no rotation possible
%T = [(FL + FR) 0 (-DeltaY*FL + DeltaY*FR)]'; % left and right propellers are situated on either side of CoM y-aix, and each generates torque along z-axis in opposite direction 

% two propellers with rotation possible along z-axis with angle alpha
alpha = 0;
DeltaX = 0.5; % distance of propeller to x coordinate of CoM. Unit: m 
T = [(FL + FR)*cos(alpha) ...
     (FL + FR)*sin(alpha) ...
     (-DeltaY*FL + DeltaY*FR)*cos(alpha) + (+DeltaX*FL + DeltaX*FR)*sin(alpha)]'; 

 DeltaT = 0.001; % in seconds
 
fg=DrawTrajectory(MRB, MA, DeltaT, m,   Xu1, Yv1, Yr1, Nr1,   Xu2, Yv2, Nr2,    Xu3, Yv3, Nr3,   T, FL, FR, alpha);
print(fg,'-dpdf','untitled');


