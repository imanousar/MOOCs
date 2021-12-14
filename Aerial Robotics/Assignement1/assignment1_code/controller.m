function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

u = 0;
g=params.gravity;
m=params.mass;
K_p  = 100;
K_v  = 10;

z = s(1);
z_u = s(2);
 
z_des = s_des(1);
z_des_u = s_des(2);

e = z_des - z;
e_dot = z_des_u - z_u;

z_des_a = 0;

u = m * ( z_des_a + K_p*e + K_v * e_dot + g  ) ;
end

