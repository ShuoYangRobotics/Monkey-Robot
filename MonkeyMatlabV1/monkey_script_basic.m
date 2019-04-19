addpath('ModernRobotics\packages\Matlab\mr');
arm_length = 1;
arm_width = 0.1;
arm_depth = 0.1;
arm_density = 100;

body_width = 0.3;
body_depth = 0.15;
body_length = 0.5;
body_density = 90;

% basic calculation
arm_mass = arm_density * arm_length * arm_depth * arm_width;
body_mass = body_density * body_length * body_depth * body_width;

arm_Ixx = 1/12*arm_mass*(arm_width*arm_width + arm_length*arm_length); 
arm_Iyy = 1/12*arm_mass*(arm_depth*arm_depth + arm_length*arm_length); 
arm_Izz = 1/12*arm_mass*(arm_width*arm_width + arm_depth*arm_depth); 


body_Ixx = 1/12*body_mass*(body_width*body_width + body_length*body_length); 
body_Iyy = 1/12*body_mass*(body_depth*body_depth + body_length*body_length); 
body_Izz = 1/12*body_mass*(body_width*body_width + body_depth*body_depth); 