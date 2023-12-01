% MATLAB controller for Webots
% File:          artificial_potential_field.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;



% 定义常量
m=3;%车的数量

alpha=40;% 计算引力需要的增益系数
betao=0.5;% 计算斥力的增益系数，都是自己设定的,障碍之间。
betaa=1;% 计算斥力的增益系数，都是自己设定的,智能体之间。
Po=0.2;%障碍影响距离，当障碍和车的距离大于这个距离时，斥力为0，即不受该障碍的影响。也是自己设定。
n=5;%障碍个数
l1=0.02;% 步长
l2=0.02;
l3=0.02;
J=1000;%循环迭代次数上限
Robstacle = 0.1;%r可能为障碍的平均半径
Rgoal=0.2;%围捕半径
Apoint=40;%指引力系数
lamad1=1;%均为指引力系数
lamad2=1;
d0=0.2;%智能体间引力斥力半径常数
d1=0.5;%智能体间力的边际常数
alphai=10;%智能体间的
Goal=[10 10];%目标点坐标
Obstacle=[-0.45 0.11;0.47 -0.23;0.38 0.8;-0.12 0.52;-0.25 -0.34];%障碍物坐标 目前有6个障碍物
zata=50;%局部最优点的判断值
flag1=0;
flag2=0;
flag3=0;
%现在计算每个车的目标点，针对不同策略目标点位置不
%其他队形
%%%

E_puck_node_two= wb_supervisor_node_get_from_def('E-pucktwo');
translation_field_two = wb_supervisor_node_get_field(E_puck_node_two, 'translation');
rotation_field_two = wb_supervisor_node_get_field(E_puck_node_two, 'rotation');
left_motor = wb_robot_get_device('left wheel motor');
right_motor = wb_robot_get_device('right wheel motor');
wb_motor_set_position(left_motor, inf);
wb_motor_set_position(right_motor, inf);
wb_motor_set_velocity(left_motor, 0);
wb_motor_set_velocity(right_motor, 0);
E_puck_node_one = wb_supervisor_node_get_from_def('E-puckone');
translation_field_one= wb_supervisor_node_get_field(E_puck_node_one, 'translation');
E_puck_node_three = wb_supervisor_node_get_from_def('E-puckthree');
translation_field_three= wb_supervisor_node_get_field(E_puck_node_three, 'translation');
ini_pos_one = wb_supervisor_field_get_sf_vec3f(translation_field_one);
ini_pos_two = wb_supervisor_field_get_sf_vec3f(translation_field_two);
ini_pos_three = wb_supervisor_field_get_sf_vec3f(translation_field_three);
E_puck_node_target = wb_supervisor_node_get_from_def('E-pucktarget');
translation_field_target= wb_supervisor_node_get_field(E_puck_node_target, 'translation')
X01(1,1)=ini_pos_one(1,1);
X01(1,2)=ini_pos_one(1,2);
X02(1,1)=ini_pos_two(1,1);
X02(1,2)=ini_pos_two(1,2);
X03(1,1)=ini_pos_three(1,1);
X03(1,2)=ini_pos_three(1,2);
X1=X01;%给出初始位置
X2=X02;
X3=X03;
TIME_STEP = 64;
step_counter = 0;
enable = 1;
pre_dis = [0, 0];
while wb_robot_step(TIME_STEP) ~= -1
  
  if mod(step_counter,1) == 0 && enable 
    now_pos_one = wb_supervisor_field_get_sf_vec3f(translation_field_one);
    now_pos_two = wb_supervisor_field_get_sf_vec3f(translation_field_two);
    now_pos_three = wb_supervisor_field_get_sf_vec3f(translation_field_three);
    X1(1,:)= now_pos_one(1,1:2) ;%给出初始位置
    X2(1,:)= now_pos_two(1,1:2) ;%
    X3(1,:)= now_pos_three(1,1:2) ;%
    cnt_temp = 0;%%%%%%%%%%%%%%%
    while norm(X2(1,:)-now_pos_two(1,1:2))<0.2
    cnt_temp = cnt_temp + 1;%%%%%%%%%%%%5
    if cnt_temp > 50
      break;
    end
    target_disp= wb_supervisor_field_get_sf_vec3f(translation_field_target);
    Goal(1,1)=target_disp(1,1);
    Goal(1,2)=target_disp(1,2);
    Goal1(1,1)=Goal(1,1)+Rgoal*cos(0);
    Goal1(1,2)=Goal(1,2)+Rgoal*sin(0);
    Goal2(1,1)=Goal(1,1)+Rgoal*cos(2*pi/3);
    Goal2(1,2)=Goal(1,2)+Rgoal*sin(2*pi/3);
    Goal3(1,1)=Goal(1,1)+Rgoal*cos(4*pi/3);
    Goal3(1,2)=Goal(1,2)+Rgoal*sin(4*pi/3);
    figure(1);
    hold on;
    plot(Goal1(1,1),Goal1(1,2),'o');
    plot(Goal2(1,1),Goal2(1,2),'o');
    plot(Goal3(1,1),Goal3(1,2),'o');
    plot(Goal(1,1),Goal(1,2),'x');
    GandOb1=[Goal1;Obstacle];
    GandOb2=[Goal2;Obstacle];
    GandOb3=[Goal3;Obstacle];
    Theta1=compute_angle(X1,GandOb1,n);%Theta 是计算出来的车和障碍，和目标之间的与X 轴之间的夹角，统一规定角度为逆时针方向，用这个模块可以计算出来。
    Theta2=compute_angle(X2,GandOb2,n);
    Theta3=compute_angle(X3,GandOb3,n);
    Angle1=Theta1(1);%Theta （1）是车和目标之间的角度，目标对车是引力。
    Angle2=Theta2(1);
    Angle3=Theta3(1);
    [Fatx1,Faty1]=compute_Attract(X1,GandOb1,alpha,Angle1,n);% 计算出目标对车的引力在x,y 方向的两个分量值。
    [Fatx2,Faty2]=compute_Attract(X2,GandOb2,alpha,Angle2,n);
    [Fatx3,Faty3]=compute_Attract(X3,GandOb3,alpha,Angle3,n);
    for i=1:n
    angle_re1(i)=Theta1(i+1);% 计算斥力用的角度，是个向量，因为有n 个障碍，就有n 个角度。
    angle_re2(i)=Theta2(i+1);
    angle_re3(i)=Theta3(i+1);
    end
    [Frerxx1,Freryy1,Fataxx1,Fatayy1]=compute_repulsion(X1,GandOb1,betao,Angle1,angle_re1,n,Po,0.5,Robstacle);%计算出斥力在x,y 方向的分量数组。
    [Frerxx2,Freryy2,Fataxx2,Fatayy2]=compute_repulsion(X2,GandOb2,betao,Angle2,angle_re2,n,Po,0.5,Robstacle);
    [Frerxx3,Freryy3,Fataxx3,Fatayy3]=compute_repulsion(X3,GandOb3,betao,Angle3,angle_re3,n,Po,0.5,Robstacle);
    %下面来计算智能体之间的力
    [Fagentx2_1,Fagenty2_1,Fagentx1_2,Fagenty1_2] = compute_agents(X1,X2,d0,d1,alphai);
    [Fagentx3_1,Fagenty3_1,Fagentx1_3,Fagenty1_3] = compute_agents(X1,X3,d0,d1,alphai);
    [Fagentx3_2,Fagenty3_2,Fagentx2_3,Fagenty2_3] = compute_agents(X2,X3,d0,d1,alphai);
    Fsumxj1=Fatx1+Frerxx1+Fataxx1+Fagentx2_1+Fagentx3_1;%x 方向的合力
    Fsumyj1=Faty1+Freryy1+Fatayy1+Fagenty2_1+Fagenty3_1;%y 方向的合力
    Fsumxj2=Fatx2+Frerxx2+Fataxx2+Fagentx1_2+Fagentx3_2;%x 方向的合力
    Fsumyj2=Faty2+Freryy2+Fatayy2+Fagenty1_2+Fagenty3_2;%y 方向的合力
    Fsumxj3=Fatx3+Frerxx3+Fataxx3+Fagentx1_3+Fagentx2_3;%x 方向的合力
    Fsumyj3=Faty3+Freryy3+Fatayy3+Fagenty1_3+Fagenty2_3;%y 方向的合力
    Far1=sqrt((Fatx1+Frerxx1+Fataxx1+Fagentx2_1+Fagentx3_1)^2+(Faty1+Freryy1+Fatayy1+Fagenty2_1+Fagenty3_1)^2);
    Far2=sqrt((Fatx2+Frerxx2+Fataxx2+Fagentx1_2+Fagentx3_2)^2+(Faty2+Freryy2+Fatayy2+Fagenty1_2+Fagenty3_2)^2);
    Far3=sqrt((Fatx3+Frerxx3+Fataxx3+Fagentx1_3+Fagentx2_3)^2+(Faty3+Freryy3+Fatayy3+Fagenty1_3+Fagenty2_3)^2);
    for k=1:n
    Rrei1(k)=(X1(1)-GandOb1(k+1,1))^2+(X1(2)-GandOb1(k+1,2))^2;
    Rrei2(k)=(X2(1)-GandOb2(k+1,1))^2+(X2(2)-GandOb2(k+1,2))^2;
    Rrei3(k)=(X3(1)-GandOb3(k+1,1))^2+(X3(2)-GandOb3(k+1,2))^2;
    end
    Pobs1=sqrt(min(min(Rrei1)));
    Pobs2=sqrt(min(min(Rrei2)));
    Pobs3=sqrt(min(min(Rrei3)));
    if (abs(Far1)<zata)
    Fesc1=Apoint/(Pobs1^lamad1+Po^lamad2);
    Fsumxj1=Fsumxj1+Fesc1*cos(Angle1);
    Fsumyj1=Fsumyj1+Fesc1*sin(Angle1);
    end
    if (abs(Far2)<zata)
    Fesc2=Apoint/(Pobs2^lamad1+Po^lamad2);
    Fsumxj2=Fsumxj2+Fesc2*cos(Angle2);
    Fsumyj2=Fsumyj2+Fesc2*sin(Angle2);
    end
    if (abs(Far3)<zata)
    Fesc3=Apoint/(Pobs3^lamad1+Po^lamad2);
    Fsumxj3=Fsumxj3+Fesc3*cos(Angle3);
    Fsumyj3=Fsumyj3+Fesc3*sin(Angle3);
    end
    Position_angle1=atan2(Fsumyj1,Fsumxj1);
    Position_angle2=atan2(Fsumyj2,Fsumxj2);
    Position_angle3=atan2(Fsumyj3,Fsumxj3);
    X1(1)=X1(1)+l1*cos(Position_angle1);
    X1(2)=X1(2)+l1*sin(Position_angle1);
    X2(1)=X2(1)+l2*cos(Position_angle2);
    X2(2)=X2(2)+l2*sin(Position_angle2);
    X3(1)=X3(1)+l3*cos(Position_angle3);
    X3(2)=X3(2)+l3*sin(Position_angle3);
    
    if (abs(X1(1)-Goal1(1,1))<0.1)&&(abs(X1(2)-Goal1(1,2))<0.1)% 是应该完全相等的时候算作到达， 还是只是接近就可以？现在按完全相等的时候编程。
        flag1=1; 
        l1=0.01;% 记录迭代到多少次，到达目标。
    else
        l1=0.02;
        flag1=0;
    end
    if (abs(X2(1)-Goal2(1,1))<0.1)&&(abs(X2(2)-Goal2(1,2))<0.1)% 是应该完全相等的时候算作到达， 还是只是接近就可以？现在按完全相等的时候编程。
        flag2=1; 
        X2(1,:)=Goal(1,:);
        l2=0.01;% 记录迭代到多少次，到达目标。
    else
        l2=0.02;
        flag2=0;
    end
    if (abs(X3(1)-Goal3(1,1))<0.1)&&(abs(X3(2)-Goal3(1,2))<0.1)% 是应该完全相等的时候算作到达， 还是只是接近就可以？现在按完全相等的时候编程。
        flag3=1; 
        l3=0.01;% 记录迭代到多少次，到达目标。
    else
        l3=0.02;
        flag3=0;
    end
    % if (flag1 ~= 0&& flag2 ~= 0&&flag3~=0)
        % break;
    % end
    if (flag2 ~= 0)
        break;
    end
    end
end
step_counter = step_counter + 1;
% Position1(K,1)=Goal1(1,1);% 把路径向量的最后一个点赋值为目标
% Position1(K,2)=Goal1(1,2);
% Position2(K,1)=Goal2(1,1);% 把路径向量的最后一个点赋值为目标
% Position2(K,2)=Goal2(1,2);
% Position3(K,1)=Goal3(1,1);% 把路径向量的最后一个点赋值为目标
% Position3(K,2)=Goal3(1,2);  
% counter = 1;
% for p=1:4:K
    % subsam_pos2(counter, 1) = Position2(p, 1);
    % subsam_pos2(counter, 2) = Position2(p, 2);
    % counter = counter + 1;
% end
% subsam_pos2(counter,1)=Goal2(1,1);% 把路径向量的最后一个点赋值为目标
% subsam_pos2(counter,2)=Goal2(1,2);% 把路径向量的最后一个点赋值为目标


SPD_KP = 3;
SPD_KI = 0.0;
SPD_KD = 0;
POS_KP = 0;
POS_KI = 0;
POS_KD = 0;
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

speed=3;
% target = [Position2(1,1),Position2(1,2),0];
angle=0;
angleTarget = 0;
counter = 1;
temp = 0;

  
  
  rotation = wb_supervisor_field_get_sf_rotation(rotation_field_two);
  displacement = wb_supervisor_field_get_sf_vec3f(translation_field_two);
  % if abs(target(1) - displacement(1)) < 0.1 && abs(target(2) - displacement(2)) < 0.1  
    % target(1) = subsam_pos2(counter, 1);
    % target(2) = subsam_pos2(counter, 2);
    % counter = counter + 1;
  % end
  target(1)=X2(1);
  target(2)=X2(2);
  % if (target(1) >= displacement(1) && target(2) >= displacement(2))
      % angle = atan(abs(target(2) - displacement(2)) / abs(target(1) - displacement(1)));
  % elseif (target(1) <= displacement(1) && target(2) >= displacement(2))
      % angle = pi - atan(abs(target(2) - displacement(2)) / abs(target(1) - displacement(1)));
  % elseif (target(1) <= displacement(1) && target(2) <= displacement(2))
      % angle = -pi + atan(abs(target(2) - displacement(2)) / abs(target(1) - displacement(1)));
  % else
      % angle = -atan(abs(target(2) - displacement(2)) / abs(target(1) - displacement(1)));
  % end
  angle=atan2(target(2) - displacement(2),target(1) - displacement(1));
  rot = atan2(displacement(2)-pre_dis(2),displacement(1)-pre_dis(1));
  % figure(1);
  % hold on;
  % plot(X2(1),X2(2),'o');
  % plot(displacement(1),displacement(2),'x');
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
  if norm(displacement(1:2)-target) > 0.1
    enable = 0;
  else
    enable = 1;
  end
  % fprintf("output= %f\n",PID_PositionalCalcOutput(speedPD));
  % disp(displacement);
  % disp(target);
  % temp = temp + 1;
  % figure(2);
  % hold on;
  % plot(target(1), target(2),'o');
  % plot(displacement(1), displacement(2),'x');
  % figure(4);
  % hold on;
  % plot(temp,rotation(4),'x');
  % figure(5);
  % hold on;
  % plot(temp,angle,'x');
  drawnow;
end  
function output = PID_PositionalCalcOutput(pid_ptr)
    pid_ptr.p_out = pid_ptr.kp * pid_ptr.err_now;
    pid_ptr.i_out = pid_ptr.ki * pid_ptr.err_all;
    % if(pid_ptr.i_out > pid_ptr.i_out_max)  %必要时可对积分限幅
    %     pid_ptr.i_out = pid_ptr.i_out_max;
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
function [Fagentx1,Fagenty1,Fagentx2,Fagenty2] = compute_agents(X1,X2,d0,d1,a1)
    %计算两智能体间的力，很显然应有两个力四个输出，两力大小相等，方向相反
    r2=(X1(1,1)-X2(1,1))^2+(X1(1,2)-X2(1,2))^2;
    r=sqrt(r2);
    if (r<d1)
        F=a1*(1/r-d0/(r^2));
    else
        F=0;
    end
    theta=atan(abs(X1(1,2)-X2(1,2))/abs(X1(1,1)-X2(1,1)));
    %下面一共有八种情况 
    if (F>=0&&X1(1,1)>=X2(1,1)&&X1(1,2)>=X2(1,1))
        Fagentx1=-F*cos(theta);
        Fagenty1=-F*sin(theta);
        Fagentx2=F*cos(theta);
        Fagenty2=F*sin(theta);
    elseif (F>=0&&X1(1,1)<=X2(1,1)&&X1(1,2)<=X2(1,1))
        Fagentx1=F*cos(theta);
        Fagenty1=F*sin(theta);
        Fagentx2=-F*cos(theta);
        Fagenty2=-F*sin(theta);
    elseif (F<0&&X1(1,1)>=X2(1,1)&&X1(1,2)>=X2(1,1))
        Fagentx1=F*cos(theta);
        Fagenty1=F*sin(theta);
        Fagentx2=-F*cos(theta);
        Fagenty2=-F*sin(theta);
    elseif (F<0&&X1(1,1)<=X2(1,1)&&X1(1,2)<=X2(1,1))
        Fagentx1=-F*cos(theta);
        Fagenty1=-F*sin(theta);
        Fagentx2=F*cos(theta);
        Fagenty2=F*sin(theta);
    elseif (F>=0&&X1(1,1)<=X2(1,1)&&X1(1,2)>=X2(1,2))
        Fagentx1=-F*cos(theta);
        Fagenty1=F*sin(theta);
        Fagentx2=F*cos(theta);
        Fagenty2=-F*sin(theta);
    elseif (F>=0&&X1(1,1)>=X2(1,1)&&X1(1,2)<=X2(1,2))
        Fagentx1=F*cos(theta);
        Fagenty1=-F*sin(theta);
        Fagentx2=-F*cos(theta);
        Fagenty2=F*sin(theta);
    elseif (F<0&&X1(1,1)<=X2(1,1)&&X1(1,2)>=X2(1,2))
        Fagentx1=F*cos(theta);
        Fagenty1=-F*sin(theta);
        Fagentx2=-F*cos(theta);
        Fagenty2=F*sin(theta);
    else
        Fagentx1=-F*cos(theta);
        Fagenty1=F*sin(theta);
        Fagentx2=F*cos(theta);
        Fagenty2=-F*sin(theta);
    end   
end
function [theta] = compute_angle(X,Xsum,n)
%Theta 是计算出来的车和障碍，和目标之间的与X 轴之间的夹角，统一规定角度为逆时针方向，用这个模块可以计算出来。
%theta数组第一行是与目标的角度，之后几行是与障碍之间的角度
    for i= 1:n+1
        %Xdist2(i)= (Xsum(i,1)-X(1))^2;%计算车与球的横坐标距离的平方
        %Ydist2(i)= (Xsum(i,2)-X(2))^2; %计算车与球的纵坐标距离的平方
        %dist(i)=sqrt(Xdist2(i)+Ydist2(i));%计算距离
        Xdist(i)= Xsum(i,1)-X(1);%横坐标差
        Ydist(i)= Xsum(i,2)-X(2);%纵坐标差
        if Xdist(i)>0 && Ydist(i)>0%1象限
            theta(i)= atan(Ydist(i)/Xdist(i));
        elseif Xdist(i)<0 && Ydist(i)>0%2象限
            theta(i)= pi-atan(Ydist(i)/abs(Xdist(i)));
        elseif Xdist(i)<0 && Ydist(i)<0%3象限
            theta(i)= pi+atan(abs(Ydist(i))/abs(Xdist(i)));
        else%其他象限
            theta(i)= atan(Ydist(i)/Xdist(i));
        end
    end
end
function [fatx,faty] = compute_Attract(X,Xsum,k,angle,n)
%计算引力
  Xdist2= (Xsum(1,1)-X(1))^2;%计算车与目标的横坐标距离的平方
  Ydist2= (Xsum(1,2)-X(2))^2;%计算车与目标的纵坐标距离的平方
  dist=sqrt(Xdist2+Ydist2);%计算距离
  fat=k*exp(-dist)*dist;%合力
  fatx=fat*cos(angle);
  faty=fat*sin(angle);
end
function [Yrerxx,Yreryy,Yataxx,Yatayy]=compute_repulsion(X,Xsum,m,angle_at,angle_re,n,Po,a,r)% 输入参数为当前坐标， Xsum 是目标和障碍的坐标向量，增益常数,障碍，目标方向的角度
  Rat=(X(1)-Xsum(1,1))^2+(X(2)-Xsum(1,2))^2;% 路径点和目标的距离平方
  rat=sqrt(Rat);% 路径点和目标的距离
  for i=1:n
    Rrei(i)=(X(1)-Xsum(i+1,1))^2+(X(2)-Xsum(i+1,2))^2;% 路径点和障碍的距离平方
    rre(i)=sqrt(Rrei(i))-r;% 路径点和障碍的距离保存在数组rrei 中
    R0=(Xsum(1,1)-Xsum(i+1,1))^2+(Xsum(1,2)-Xsum(i+1,2))^2;
  %r0=sqrt(R0)-r;%以上为目标与障碍之间的距离 或许是提醒目标与障碍过近
    if rre(i)>Po% 如果每个障碍和路径的距离大于障碍影响距离，斥力令为0
      Yrerx(i)=0;
      Yrery(i)=0;
      Yatax(i)=0;
      Yatay(i)=0;
    else
      Yrer(i)=m*(1/rre(i)-1/Po)*((1/rre(i))^2)*(rat^0.5);% 分解的Fre1 向量
      Yata(i)=a*a*m*((1/rre(i)-1/Po)^2)*(rat^(-0.5));% 分解的Fre2 向量
      Yrerx(i)=-Yrer(i)*cos(angle_re(i));%angle_re(i)=Y(i+1)
      Yrery(i)=-Yrer(i)*sin(angle_re(i));
      Yatax(i)=Yata(i)*cos(angle_at);%angle_at=Y(1)
      Yatay(i)=Yata(i)*sin(angle_at);
    end
  end%判断距离是否在障碍影响范围内
  Yrerxx=sum(Yrerx);% 叠加斥力的分量
  Yreryy=sum(Yrery);
  Yataxx=sum(Yatax);
  Yatayy=sum(Yatay);
end
