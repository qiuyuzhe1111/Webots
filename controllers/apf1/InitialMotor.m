function [left_motor, right_motor] = InitialMotor()
    left_motor = wb_robot_get_device('left wheel motor');
    right_motor = wb_robot_get_device('right wheel motor');
    wb_motor_set_position(left_motor, inf);
    wb_motor_set_position(right_motor, inf);
    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);
end