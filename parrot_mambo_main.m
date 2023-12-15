clc 
clear
close all

%% Default part
global Ts m g Kv

Ts = 0.01 ;     % [s] Sample time 
m = 0.075 ;     % [kg] Drone mass
g = 9.81 ;      % [m.s-2] Gravity acceleration
Kv = 0.00887 ;  % [-] Air friction 

GLOBAL_PARAMETERS = [Ts m g Kv] ;
initial_pos_conditions_M1 = [0 0 0] ;

% Parot MAMBO drone model (simulation) parameters
angle_sat = 0.22 ;      % [rad] Saturation of theta and phi
delay = 0.08 ;          % [s] Communication delay 
Tmin = 0.4786 ;         % [N] Saturation in T = 1*0.0981/0.37 //T=0.0981/0.37*uT+0.7437 
Tmax = 1.0088;          % [N] Maximal thrust
error_m = 1.05;          % [%] 5% error on mass/thrust calibration
theta_biais = 0.005 ;   % [rad]
phi_biais = 0.005 ;     % [rad]

variance_qualysis  = 1e-6 ;
variance_boucle_attitude =  1e-7  ;

% Dynamic model of thrust 
Ac_thrust = [0.92] ;
Bc_thrust = [0.04 1] ;
sysc_thrust = tf(Ac_thrust, Bc_thrust) ;
sysd_thrust = c2d(sysc_thrust,Ts) ;
B_thrust = sysd_thrust.num ;      % BT{1} numerator of drone pitch/roll model 
A_thrust = sysd_thrust.den ;      % AT{1} den of drone pitch/roll model
%step(sysc)

% Dynamic model of control angles (2nd order transfer function)
w_angles = 10 ; % Natural frequency
ksi_angles = 0.35 ; % Damping factor
K_angles = 1 ;
Ac_angles = [K_angles] ;
Bc_angles = [(1/w_angles^2) 2*ksi_angles/w_angles 1] ;
sysc_angles = tf(Ac_angles,Bc_angles) ;
sysd_angles = c2d(sysc_angles,Ts) ;
B_angles = sysd_angles.num; % B{1} numerator of drone pitch/roll model 
A_angles = sysd_angles.den; % A{1} den of drone pitch/roll model
% step(sysc_angles)

% Dynamic model of yaw
Ac_yaw = [1] ;
Bc_yaw = [0.4 1] ;
sysc_yaw = tf(Ac_yaw,Bc_yaw) ;
sysd_yaw = c2d(sysc_yaw, Ts) ;
B_yaw = sysd_yaw.num ;       % B{1} numerator of drone pitch/roll model 
A_yaw = sysd_yaw.den ;       % A{1} den of drone pitch/roll model
%step(sysc)


%% Testing parameters
x0 = -1.18;
y0 = -1.21;
z0 = 0.8;
task_ped = 5;

%% Linear state space representation
A = [zeros(3) diag([1 1 1]); zeros(3) zeros(3)];
B_sub = [0 g 0 0; -g 0 0 0; 0 0 0 1/m];
B = [zeros(3, 4); B_sub];
C = [diag([1 1 1]) zeros(3); zeros(3) diag([1 1 1])];
D = zeros(6, 4);
ML = ss(A, B, C, D);

Q_gain_lin = diag([1 1 1 1 1 1]);
R_gain_lin = diag([30 30 1 15]); % less aggressive
K_gain_lin = lqr(A, B, Q_gain_lin, R_gain_lin);

%% Discrete linear state representation

MLD = c2d(ML, Ts);
Ad = MLD.A;
Bd = MLD.B;
Cd = MLD.C;
Dd = MLD.D;

%%
% Q_gain = diag([1 1 1 1 1 1]);
% R_gain = diag([1 1 1 1]);
K_gain = dlqr(Ad, Bd, Q_gain_lin, R_gain_lin);

Ki_x = 0.04;
Ki_y = 0.04;
Ki_z = 0.023; %0.023;
sat_z = 1;

%% Animation
plot_open_loop = false;

if (plot_open_loop)
    fprintf('Plot open loop\n');
    % Plot drone position wrt step inputs with no controller
    final_pos_x = out.position.Data(length(out.position.Data(:,1)),1);
    final_pos_y = out.position.Data(length(out.position.Data(:,2)),2);
    final_pos_z = out.position.Data(length(out.position.Data(:,3)),3);

    plot3(out.position.Data(:,1),out.position.Data(:,2),out.position.Data(:,3));
    xlabel('x');ylabel('y');zlabel('z');
    arm_length = 0.25;
    drone_config = [2*arm_length -2*arm_length 0 0 0; 0 0 0 2*arm_length -2*arm_length; 0 0 0 0 0];
    update_drone_x = drone_config(1,:) + final_pos_x;
    update_drone_y = drone_config(2,:) + final_pos_y;
    update_drone_z = drone_config(3,:) + final_pos_z;
    update_drone = [update_drone_x; update_drone_y; update_drone_z];
    RotatedDrone = update_drone;
    % Main plot for 3D simulation
%     droneView = subplot(4,4,[1 2 3 5 6 7 9 10 11]);
    hold on
    p = plot3(RotatedDrone(1,:,1),RotatedDrone(2,:,1),RotatedDrone(3,:,1),'k.-');
%     axisLim = 30;
%     axis([-axisLim axisLim -axisLim axisLim -axisLim axisLim])
    grid on;
    grid minor;
else
    fprintf('Do not plot open loop\n');
end

if (false)

%     k = 0;              % Counter
%     f1 = figure('units','normalized','outerposition',[0 0 1 1]);
%     set(f1,'WindowKeyPressFcn',@KeyDown, 'WindowKeyReleaseFcn', @KeyUp); 
%     % Drone shape definition
%     Drone = [ 1 -1 0 0 0        
%               0 0 0 1 -1        
%               0 0 0 0 0];       
%     RotatedDrone = Drone;
%     % Main plot for 3D simulation
% %     droneView = subplot(4,4,[1 2 3 5 6 7 9 10 11]);
%     p = plot3(RotatedDrone(1,:,1),RotatedDrone(2,:,1),RotatedDrone(3,:,1),'k.-');
%     animLine = animatedline('MaximumNumPoints',1000,...
%                             'Color','r');
%     xlabel('x');ylabel('y');zlabel('z')
%     axisLim = 30;
%     axis([-axisLim axisLim -axisLim axisLim -axisLim axisLim])
%     grid on;
%     grid minor;
else
%     fprintf('Do not show simulink\n');
end

sim("mambo_offline_simulator.slx");
run("parrot_mambo_plot.m");