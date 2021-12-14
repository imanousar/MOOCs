function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

x = state.pos(1);
y = state.pos(2);
z = state.pos(3);

x_dot = state.vel(1); 
y_dot = state.vel(2);
z_dot = state.vel(3);

phi = state.rot(1);
theta = state.rot(2);
psi = state.rot(3);

p = state.omega(1);
q = state.omega(2);
r = state.omega(3);

% parameters
m = params.mass; 
I = params.I;
inv_I = params.invI; 
g = params.gravity;
arm_len = params.arm_length; 

minF = params.minF;
maxF = params.maxF;

% desired state
x_des = des_state.pos(1);
y_des = des_state.pos(2);
z_des = des_state.pos(3);

x_des_dot = des_state.vel(1);
y_des_dot = des_state.vel(2);
z_des_dot = des_state.vel(3);

x_des_ddot = des_state.acc(1);
y_des_ddot = des_state.acc(2);
z_des_ddot = des_state.acc(3);

yaw_des =  des_state.yaw;
yaw_des_dot = des_state.yawdot;

% Constants
K_dx = 20;
K_px = 2;
K_dy = 20;
K_py = 2;
K_pz = 700;
K_dz = 20;
K_pphi = 150;
K_dphi = 2;
K_ptheta = 150;
K_dtheta = 2;
K_ppsi = 150;
K_dpsi = 2;

% Commanded accelerations
commanded_r_ddot_x = x_des_ddot + K_dx * (x_des_dot - x_dot) + K_px * (x_des - x);
commanded_r_ddot_y = y_des_ddot + K_dy * (y_des_dot - y_dot) + K_py * (y_des - y);
commanded_r_ddot_z = z_des_ddot + K_dz * (z_des_dot - z_dot) + K_pz * (z_des - z);

% Thrust
F = m * (g + commanded_r_ddot_z);

% Moment
p_des = 0;
q_des = 0;
r_des = yaw_des_dot;
phi_des = 1/g * (commanded_r_ddot_x * sin(psi_des) - commanded_r_ddot_y * cos(psi_des));
theta_des = 1/g * (commanded_r_ddot_x * cos(psi_des) + commanded_r_ddot_y * sin(psi_des));
M = [K_pphi * (phi_des - phi) + K_dphi * (p_des - p);
     K_ptheta * (theta_des - theta) + K_dtheta * (q_des - q);
     K_ppsi * (psi_des - psi) + K_dpsi * (r_des - r)];
% =================== Your code ends here ===================

end
