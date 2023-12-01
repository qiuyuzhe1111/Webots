TIME_STEP = 64;

MAX_SPEED = 6.28;

% get a handler to the motors and set target position to infinity (speed control)
left_motor = wb_robot_get_device('left wheel motor');
right_motor = wb_robot_get_device('right wheel motor');
wb_motor_set_position(left_motor, inf);
wb_motor_set_position(right_motor, inf);

% set up the motor speeds at 10% of the MAX_SPEED.
% wb_motor_set_velocity(left_motor, 0.1 * MAX_SPEED);
% wb_motor_set_velocity(right_motor, 0.1 * MAX_SPEED);
wb_motor_set_velocity(left_motor, 0.0 * MAX_SPEED);
wb_motor_set_velocity(right_motor, 0.0 * MAX_SPEED);

while wb_robot_step(TIME_STEP) ~= -1
E_puck_node = wb_supervisor_node_get_from_def('E-pucktwo');
translation_field = wb_supervisor_node_get_field(E_puck_node, 'translation');
displacement = wb_supervisor_field_get_sf_vec3f(translation_field);
disp(displacement);
end
