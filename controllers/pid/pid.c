/*
 * File:          pid_controller.c
 * Date:
 * Description:
 * Author:        RobotFreak
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <math.h>
#include <webots/supervisor.h>
//#include "pid.h"
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define SPD_KP 5
#define SPD_KI 1
#define SPD_KD 0.1
#define POS_KP 0
#define POS_KI 0
#define POS_KD 0
#define SPD_OUT_MAX 1
#define SPD_I_MAX 1
#define pi 3.1415927
//#define speed 1
typedef struct
{   
    double kp;           //比例系数
    double ki;           //积分系数
    double kd;           //微分系数
    double err_all;      //误差和
    double err_now;      //当前误差
    double err_last;     //上次误差
    double err_llast;    //上上次误差
    double p_out;        //比例输出
    double i_out;        //积分输出
    double d_out;        //微分输出
    double output;       //输出量
    double i_out_max;    //积分限幅
    double out_max;      //输出限幅
    double delta_output; //输出量增量
}PID;

void PID_Init(PID* pid_ptr, double kp, double ki, double kd, double output_max, double i_max)
{
    pid_ptr->kp = kp;
    pid_ptr->ki = ki;
    pid_ptr->kd = kd;
    pid_ptr->err_now = 0;
    pid_ptr->err_last = 0;
    pid_ptr->err_llast = 0;
    pid_ptr->err_all = 0;
    pid_ptr->p_out = 0;
    pid_ptr->i_out = 0;
    pid_ptr->d_out = 0;
    pid_ptr->delta_output = 0;
    pid_ptr->output = 0;
    pid_ptr->i_out_max = i_max;
    pid_ptr->out_max = output_max;
}

void PID_ErrUpdate(PID* pid_ptr, double target, double real)
{
    double err = target - real;  //期望值-实际值得到误差
    pid_ptr->err_llast = pid_ptr->err_last;
    pid_ptr->err_last = pid_ptr->err_now;
    pid_ptr->err_now = err;
    pid_ptr->err_all += err;
}

double PID_PositionalCalcOutput(PID* pid_ptr)
{
    pid_ptr->p_out = pid_ptr->kp * pid_ptr->err_now;
    pid_ptr->i_out = pid_ptr->ki * pid_ptr->err_all;
    // if(pid_ptr->i_out > pid_ptr->i_out_max)  //必要时可对积分限幅
    //     pid_ptr->i_out = pid_ptr->i_out_max;
    pid_ptr->d_out = pid_ptr->kd * (pid_ptr->err_now - pid_ptr->err_last);
    pid_ptr->output = pid_ptr->p_out + pid_ptr->i_out + pid_ptr->d_out;
    if(pid_ptr->output > pid_ptr->out_max)   //必要时可对输出限幅
      pid_ptr->output = pid_ptr->out_max;
    if(pid_ptr->output < -1 * pid_ptr->out_max)
      pid_ptr->output = -1 * pid_ptr->out_max;
    return pid_ptr->output;
}

double PID_IncrementalCalcOutput(PID* pid_ptr)
{
    pid_ptr->delta_output = pid_ptr->kp * (pid_ptr->err_now - pid_ptr->err_last) + pid_ptr->ki * pid_ptr->err_now + pid_ptr->kd * (pid_ptr->err_now - 2*pid_ptr->err_last + pid_ptr->err_llast);
    return pid_ptr->delta_output;
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
    //获取deviceTag
  //WbDeviceTag motor = wb_robot_get_device("E-puck");
  WbNodeRef E_puck_node = wb_supervisor_node_get_from_def("E-puckone");
  WbFieldRef translation_field = wb_supervisor_node_get_field(E_puck_node, "translation");
  WbFieldRef rotation_field = wb_supervisor_node_get_field(E_puck_node, "rotation");
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

 // WbDeviceTag positionSensor = wb_robot_get_device("positionSensor");
  //wb_position_sensor_enable(positionSensor, TIME_STEP); //位置传感器使能
  //PID结构体
  PID speedPD; //p,i,d看作三个独立的单词(缩写)
  PID_Init(&speedPD, SPD_KP, SPD_KI, SPD_KD, SPD_OUT_MAX, SPD_I_MAX);//虽然写的速度 实际上控制角度
  //位置(rad)和速度(rad/s)
  //double position = 0;
 // double positionLast = 0;
  double speed = 5;

  // double speedTarget = 5.0;
  // double *rotation;
  double target[3]={1,1,0};
  // double *displacement;
  double angle = 0;
  double angleTarget = 0;
 
  
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
    const double *rotation = wb_supervisor_field_get_sf_rotation(rotation_field);
    const double *displacement = wb_supervisor_field_get_sf_vec3f(translation_field);
    /*
    下面计算角度  弧度制
    */
    if (target[0]>=displacement[0] && target[1]>=displacement[1]){
    angle = atan(fabs(target[1]-displacement[1])/fabs(target[0]-displacement[0]));
    }
    else if (target[0]<=displacement[0] && target[1]>=displacement[1]){
    angle = pi - atan(fabs(target[1]-displacement[1])/fabs(target[0]-displacement[0]));
    }
    else if (target[0]<=displacement[0] && target[1]<=displacement[1]){
    angle = pi + atan(fabs(target[1]-displacement[1])/fabs(target[0]-displacement[0]));
    }
    else{
    angle = atan(fabs(target[1]-displacement[1])/fabs(target[0]-displacement[0]));
    }
    angle = rotation[3]-angle;
    //angle正，应向左转；若为负向右转
    //positionLast = position;
   // position = wb_position_sensor_get_value(positionSensor);
    //speed = (position - positionLast) / TIME_STEP * 1000; //TIME_STEP是毫秒

    /* Process sensor data here */
    PID_ErrUpdate(&speedPD, angleTarget, angle);
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
    // printf("speedout: %f\n", PID_PositionalCalcOutput(&speedPD));
    //printf("force: %f\n", PID_PositionalCalcOutput(&speedPD));
    // wb_motor_set_force(left_motor, speed - PID_PositionalCalcOutput(&speedPD));
    // wb_motor_set_force(right_motor, speed + PID_PositionalCalcOutput(&speedPD));
    wb_motor_set_velocity(left_motor, speed - PID_PositionalCalcOutput(&speedPD));
    wb_motor_set_velocity(right_motor, speed + PID_PositionalCalcOutput(&speedPD));
  }

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

