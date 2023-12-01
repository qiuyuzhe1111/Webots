% MATLAB controller for Webots
% File:          target_control.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 64;
target_node = wb_supervisor_node_get_from_def('E-pucktarget');

    left_motor = wb_robot_get_device('left wheel motor');
    right_motor = wb_robot_get_device('right wheel motor');
    wb_motor_set_position(left_motor, inf);
    wb_motor_set_position(right_motor, inf);
    wb_motor_set_velocity(left_motor, 0);
    wb_motor_set_velocity(right_motor, 0);
% get and enable devices, e.g.:
%  camera = wb_robot_get_device('camera');
%  wb_camera_enable(camera, TIME_STEP);
%  motor = wb_robot_get_device('motor');
position=[-0.01 0;0.403 0.09;0.724 0.377;0.643 -0.814];
% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
count = 1;
pre_dis = [0, 0];
while wb_robot_step(TIME_STEP) ~= -1
SPD_KP = 2;
SPD_KI = 1;
SPD_KD = 0.5;
SPD_OUT_MAX = 1;
SPD_I_MAX = 1;
% 定义PID结构体
speedPD.kp = SPD_KP;
speedPD.ki = SPD_KI;
speedPD.kd = SPD_KD;
speedPD.err_now = 0;
speedPD.err_last = 0;
speedPD.err_llast = 0;
speedPD.err_all = 0;
speedPD.p_out = 0;
speedPD.i_out = 0;
speedPD.d_out = 0;
speedPD.delta_output = 0;
speedPD.output = 0;
speedPD.i_out_max = SPD_I_MAX;
speedPD.out_max = SPD_OUT_MAX;

% 主程序

speed=1.3;
angle=0;
angleTarget = 0;
counter = 1;
temp = 0;
displacement = GetPuckState(target_node);
 target = position(count,:);
  angle=atan2(target(2) - displacement(2),target(1) - displacement(1));
  rot = atan2(displacement(2)-pre_dis(2),displacement(1)-pre_dis(1));
  % angle = rotation(4) - angle;
  angle = rot - angle;
  pre_dis = displacement(1:2);
  if angle < -pi
    angle = angle + 2*pi;
    elseif angle > pi
    angle = angle - 2*pi;
  end
  % fprintf("angle = %f\n",angle);
  % fprintf("target(1) = %f\ttarget(2) = %f\n",target(1), target(2));
  speedPD = PID_ErrUpdate(speedPD, angleTarget, angle);   
  wb_motor_set_velocity(left_motor, speed - PID_PositionalCalcOutput(speedPD));
  wb_motor_set_velocity(right_motor, speed + PID_PositionalCalcOutput(speedPD));
  % read the sensors, e.g.:
  %  rgb = wb_camera_get_image(camera);

  % Process here sensor data, images, etc.

  % send actuator commands, e.g.:
  %  wb_motor_set_postion(motor, 10.0);

  % if your code plots some graphics, it needs to flushed like this:
  if norm(displacement - target) < 0.1
  count = count + 1;
      if count > 4
      count = 1;
      end
 end
  drawnow;

end
function output = PID_PositionalCalcOutput(pid_ptr)
    pid_ptr.p_out = pid_ptr.kp * pid_ptr.err_now;
    pid_ptr.i_out = pid_ptr.ki * pid_ptr.err_all;
    if(pid_ptr.i_out > pid_ptr.i_out_max)  %必要时可对积分限幅
        pid_ptr.i_out = pid_ptr.i_out_max;
    end
    pid_ptr.d_out = pid_ptr.kd * (pid_ptr.err_now - pid_ptr.err_last);
    % fprintf("p_out: %f\ti_out: %f\td_out: %f\n", pid_ptr.p_out, pid_ptr.i_out, pid_ptr.d_out);
    pid_ptr.output = pid_ptr.p_out + pid_ptr.i_out + pid_ptr.d_out;
    if(pid_ptr.output > pid_ptr.out_max)   %必要时可对输出限幅
        pid_ptr.output = pid_ptr.out_max;
    end
    if(pid_ptr.output < -1 * pid_ptr.out_max)
        pid_ptr.output = -1 * pid_ptr.out_max;
    end
    output = pid_ptr.output;
end
function output = PID_ErrUpdate(pid_ptr, target, real)
    err = target - real;  %期望值-实际值得到误差
    pid_ptr.err_llast = pid_ptr.err_last;
    pid_ptr.err_last = pid_ptr.err_now;
    pid_ptr.err_now = err;
    pid_ptr.err_all = pid_ptr.err_all + err;
    output = pid_ptr;
end
% cleanup code goes here: write data to files, etc.