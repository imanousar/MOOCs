function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

u1 = 0;
u2 = 0;

%parameters 
Kpz=120;
Kvz=10;
Kpphi=1000;
Kvphi=40;
Kpy=50;
Kvy=10;

% constants 
m = params.mass;
g = params.gravity;
Ixx = params.Ixx; 

%current 
y = state.pos(1);
z = state.pos(2);
y_dot = state.vel(1);
z_dot = state.vel(2);
phi = state.rot(1);
phi_dot = state.omega(1);

%desired
y_T = des_state.pos(1);
z_T = des_state.pos(2);
y_T_dot = des_state.vel(1);
z_T_dot = des_state.vel(2);
y_T_ddot =  des_state.acc(1);
z_T_ddot =  des_state.acc(2);

phi_T_ddot = 0;
phi_T_dot = 0;

phi_c = -1/g*(y_T_ddot+Kvy*(y_T_dot-y_dot)+Kpy*(y_T-y));
u2 = Ixx*(phi_T_ddot+Kvphi*(phi_T_dot-phi_dot)+Kpphi*(phi_c-phi)); 
u1 = m*(g+z_T_ddot+Kvz*(z_T_dot-z_dot)+Kpz*(z_T-z));


end

