%获取车,障碍物目标及数量
[pucks,puck_num] = LoadPucks();
[obstacles,obs_num] = LoadObstacles();
target_puck = wb_supervisor_node_get_from_def('E-pucktarget');
[left_motor, right_motor] = InitialMotor();

obstacle_state = GetObstacleState(obstacles);
self_index = GetSelfPuck();
m=3;%车的数量
%起始坐标
alpha=20;% 计算引力需要的增益系数
betao=0.5;% 计算斥力的增益系数，都是自己设定的,障碍之间。
betaa=2;% 计算斥力的增益系数，都是自己设定的,智能体之间。
Po=0.2;%障碍影响距离，当障碍和车的距离大于这个距离时，斥力为0，即不受该障碍的影响。也是自己设定。
n=5;%障碍个数
l=0.02;% 步长
J=1000;%循环迭代次数上限
Robstacle = 0.05;%r可能为障碍的平均半径
Rgoal=0.15;%围捕半径
Apoint=50;%指引力系数
lamad1=1;%均为指引力系数
lamad2=1;
d0=0.2;%智能体间引力斥力半径常数
d1=0.5;%智能体间力的边际常数
alphai=35;%智能体间的
% Goal=[10 10];%目标点坐标
zata=50;%局部最优点的判断值
flag1=0;
flag2=0;
flag3=0;
%现在计算每个车的目标点，针对不同策略目标点位置不同
%围捕队形
TIME_STEP = 64;
step_counter = 0;
enable = 1;
pre_dis = [0, 0];
goal = zeros([puck_num,2]);
rer_limit = 5;
past_position = GetPuckState(pucks);
past_target = GetPuckState(target_puck);
speed_able = 1;
pic_posi = [];
pic_target = [];
pic_count = 1;
while wb_robot_step(TIME_STEP) ~= -1
  if mod(step_counter,1) == 0 && enable 
    puck_position = GetPuckState(pucks);
    current_position = puck_position;
    cnt_temp = 0;%%%%%%%%%%%%%%%\
    cnt_stop = 0;
   
    while norm(current_position(self_index,1:2)-puck_position(self_index,1:2))<0.2
      cnt_temp = cnt_temp + 1;%%%%%%%%%%%%
      if cnt_temp > 5
        break;
      end
      if norm(past_position(self_index,1:2) - current_position(self_index,1:2))<0.1
        cnt_stop = cnt_stop + 1;
      end
      if cnt_stop > 5
        break;
      end
    target_position= GetPuckState(target_puck);
    mov_angle = atan2(target_position(2) - past_target(2),target_position(1) - past_target(1));
    mov_angle2 = pi + atan2(target_position(2) - past_target(2),target_position(1) - past_target(1));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     for i = 1:puck_num
% %       goal1(i,:) = target_position + Rgoal * [cos((i-1)*2*pi/puck_num) sin((i-1)*2*pi/puck_num)];
%        goal(i,:) = target_position + Rgoal * [cos((i-1)*2*pi/puck_num) sin((i-1)*2*pi/puck_num)];%围捕队形
%     end
       goal(3,:) = target_position + Rgoal * [cos(mov_angle+pi/4) sin(mov_angle+pi/4)];
       goal(2,:) = target_position + Rgoal * [cos(mov_angle) sin(mov_angle)];
       goal(1,:) = target_position + Rgoal * [cos(mov_angle-pi/4) sin(mov_angle-pi/4)];
       % goal(1,:) = target_position + Rgoal * [cos(mov_angle2+pi/4) sin(mov_angle2+pi/4)];
       % goal(3,:) = target_position + Rgoal * [cos(mov_angle2) sin(mov_angle2)];
       % goal(2,:) = target_position + Rgoal * [cos(mov_angle2-pi/4) sin(mov_angle2-pi/4)];
%     for i = 1:puck_num
%         dis(i) = norm(target_position - puck_position(i,:));
%     end
%     if dis(1) > Rgoal*1.1 && dis(2) > Rgoal*1.1&& dis(3) > Rgoal*1.1
%     sum(1) = norm(puck_position(1)-goal1(1))+norm(puck_position(2)-goal1(2))+norm(puck_position(3)-goal1(3));
%     sum(2) = norm(puck_position(1)-goal1(1))+norm(puck_position(2)-goal1(3))+norm(puck_position(3)-goal1(2));
%     sum(3) = norm(puck_position(1)-goal1(2))+norm(puck_position(2)-goal1(3))+norm(puck_position(3)-goal1(1));
%     sum(4) = norm(puck_position(1)-goal1(2))+norm(puck_position(2)-goal1(1))+norm(puck_position(3)-goal1(3));
%     sum(5) = norm(puck_position(1)-goal1(3))+norm(puck_position(2)-goal1(2))+norm(puck_position(3)-goal1(1));
%     sum(6) = norm(puck_position(1)-goal1(3))+norm(puck_position(2)-goal1(1))+norm(puck_position(3)-goal1(2));
%     [summin,len_num] = min(sum);
%     if len_num == 1
%         goal(1,:)=goal1(1,:);
%         goal(2,:)=goal1(2,:);
%         goal(3,:)=goal1(3,:);
%     elseif len_num == 2
%         goal(1,:)=goal1(1,:);
%         goal(2,:)=goal1(3,:);
%         goal(3,:)=goal1(2,:);
%     elseif len_num == 3  
%         goal(1,:)=goal1(2,:);
%         goal(2,:)=goal1(3,:);
%         goal(3,:)=goal1(1,:);
%     elseif len_num == 4  
%         goal(1,:)=goal1(2,:);
%         goal(2,:)=goal1(1,:);
%         goal(3,:)=goal1(3,:);  
%     elseif len_num == 5  
%         goal(1,:)=goal1(3,:);
%         goal(2,:)=goal1(2,:);
%         goal(3,:)=goal1(1,:);
%     else
%         goal(1,:)=goal1(3,:);
%         goal(2,:)=goal1(1,:);
%         goal(3,:)=goal1(2,:);
%     end
%     end
%     goalnum = Cal_posi_goal(puck_position, goal1);
%     for i =  1:puck_num
%         goal(i,:) = goal1(goalnum(i),:);
%     endg
%     figure(1);
%     hold on;
%     plot(goal(self_index,:),'o');
    %其他队形
    %%%
    force = CalculateForce(current_position,goal,obstacle_state,alpha,betao,Po,0.5,Robstacle,d0,d1,alphai,rer_limit,0.15,target_position,1000);
    force_angle = CalculateAngle(force,current_position,goal,obstacle_state,zata,Apoint,lamad1,lamad2,Po);
    for i = 1:puck_num
      current_position(i,:) = current_position(i,:) + l*[cos(force_angle(i)) sin(force_angle(i))];
    end
    if (norm(current_position(self_index,:)-goal(self_index,:)) < 0.3)% 是应该完全相等的时候算作到达， 还是只是接近就可以？现在按完全相等的时候编程。
        current_position(self_index,:) = goal(self_index,:);
        break;
    end
%     
    end
    past_position = current_position;
    past_target = target_position;
  end

SPD_KP = 2;
SPD_KI = 0;
SPD_KD = 0.1;
SPD_OUT_MAX = 3;
SPD_I_MAX = 3;
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

speed=2.5;%围捕是3
angle=0;
angleTarget = 0;
counter = 1;
temp = 0;
  displacement = GetPuckState(pucks{self_index});
  pic_posi(pic_count,:) = displacement(1:2);
  pic_target(pic_count,:) = target_position(1:2);
  pic_count = pic_count + 1;
  target = current_position(self_index,:);
  for i = 1:obs_num
      if norm(target - obstacle_state(i,:)) < 0.1
          angle_goal_self = atan2(displacement(2)- goal(self_index,2),displacement(1)- goal(self_index,1));
          angle_obs_self = atan2(displacement(2)- obstacle_state(i,2),displacement(1)- obstacle_state(i,1));
          angle_tar_obs = atan2(target_position(2)-obstacle_state(i,2),target_position(1)-obstacle_state(i,1));
          angle_goal_obs = atan2(goal(self_index,2)-obstacle_state(i,2),goal(self_index,1)-obstacle_state(i,1));
          if abs(angle_goal_self - angle_obs_self) < pi/6
            delta = angle_obs_self - angle_tar_obs;
            if delta < 0
              delta = delta + 2*pi;
            end
            if delta > pi 
              delta = 2*pi - delta;
              delta = - delta;
            end
            delta = delta / 2;
            delta = delta - angle_obs_self;
            target(1,1) = obstacle_state(i,1) + 0.1 * cos(delta);
            target(1,2) = obstacle_state(i,2) + 0.1 * sin(delta);
            % elseif delta < -pi
              % delta = delta +2*pi;
            % end
            
          else
          target(1,1) = obstacle_state(i,1) + 0.1 * cos(angle_goal_obs );
          target(1,2) = obstacle_state(i,2) + 0.1 * sin(angle_goal_obs );
          end
          % angle_tar_obs = atan2(target_position(2)-obstacle_state(i,2),target_position(1)-obstacle_state(i,1));
          % target(1,1) = target(1,1) + 0.2 * cos(angle_tar_obs);
          % target(1,2) = target(1,2) + 0.2 * sin(angle_tar_obs);tate(i,2),target_position(1)-obstacle_state(i,1));

      end
  end
  % figure(1);
  % hold on;
  % plot(target(1),target(2),'x');
  % plot(obstacle_state(:,1),obstacle_state(:,2),'o');
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
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for i = 1:puck_num
       dis(i) = norm(goal(i,:) - puck_position(i,:));
  end
   
  if norm(target_position(1:2) - displacement(1:2)) < Rgoal * 1.1 && dis(1) < 0.05 && dis(2) < 0.05&& dis(3) <0.05
      %Ispeed = wb_supervisor_node_get_velocity(target_puck);

     speed = 1.3;     % speed = sqrt(Ispeed(1)^2 + Ispeed(2)^2);
  else
      speed = 3;
  end
  speedPD = PID_ErrUpdate(speedPD, angleTarget, angle);   
  wb_motor_set_velocity(left_motor, speed - PID_PositionalCalcOutput(speedPD));
  wb_motor_set_velocity(right_motor, speed + PID_PositionalCalcOutput(speedPD));
  if norm(displacement(1:2)-target) > 0.2
    enable = 0;
  else
    enable = 1;
  end
  drawnow;
  if self_index == 1
   save position_data1.mat pic_posi;
  elseif self_index == 2
   save position_data2.mat pic_posi;   
  else 
   save position_data3.mat pic_posi; 
  end
  save target_data.mat pic_target;
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
% function position_minnum = Cal_posi_goal(displacement, goal) 
%     for j = 1:3
%         a(1) = norm(displacement(j,:) - goal(1,:));
%         a(2) = norm(displacement(j,:) - goal(2,:));
%         a(3) = norm(displacement(j,:) - goal(3,:));
%         amin = min(a);
%         if amin == a(1)
%         	position_minnum(j) = 1;
%         elseif amin == a(2)
%             position_minnum(j) = 2;
%         else
%             position_minnum(j) = 3;
%         end
%         
%     end
% end
% function long = Cal_all_len(puck_position, goal)
%     sum(1) = norm(puck_position(1)-goal(1))+norm(puck_position(2)-goal(2))+norm(puck_position(3)-goal(3));
%     sum(2) = norm(puck_position(1)-goal(1))+norm(puck_position(2)-goal(3))+norm(puck_position(3)-goal(2));
%     sum(3) = norm(puck_position(1)-goal(2))+norm(puck_position(2)-goal(3))+norm(puck_position(3)-goal(1));
%     sum(4) = norm(puck_position(1)-goal(2))+norm(puck_position(2)-goal(1))+norm(puck_position(3)-goal(3));
%     sum(5) = norm(puck_position(1)-goal(3))+norm(puck_position(2)-goal(2))+norm(puck_position(3)-goal(1));
%     sum(6) = norm(puck_position(1)-goal(3))+norm(puck_position(2)-goal(1))+norm(puck_position(3)-goal(2));
%     
% end