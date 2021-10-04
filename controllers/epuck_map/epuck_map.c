#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

#define WHEEL_DISTANCE 0.052    // Distance between wheels in meters (axis length); it's the same value as "WHEEL_SEPARATION" but expressed in meters.
#define WHEEL_RADIUS 0.0205    // m.
#define WHEEL_CIRCUMFERENCE WHEEL_RADIUS*M_PI*2.0    // Wheel circumference (meters).
#define ROBOT_RADIUS 0.035 // meters.
#define RANGE_MIN 0.005+ROBOT_RADIUS // 0.5 cm + ROBOT_RADIUS.
#define RANGE_MAX 0.05+ROBOT_RADIUS // 5 cm + ROBOT_RADIUS. 

// e-puck proximity positions (cm), x pointing forward, y pointing left
//           P7(3.5, 1.0)   P0(3.5, -1.0)
//       P6(2.5, 2.5)           P1(2.5, -2.5)
//   P5(0.0, 3.0)                   P2(0.0, -3.0)
//       P4(-3.5, 2.0)          P3(-3.5, -2.0)

// e-puck proximity orentations (degrees)
//           P7(10)   P0(350)
//       P6(40)           P1(320)
//   P5(90)                   P2(270)
//       P4(160)          P3(200)
	
bool detect_obstacle_ahead(float d[8]) {
  return ( (d[0] < RANGE_MAX) || 
           (d[1] < RANGE_MAX) || 
           (d[6] < RANGE_MAX) || 
           (d[7] < RANGE_MAX));
}

float convert_intensity_to_meters(float prox) {
  float dist = 0.0;
  if (prox > 0.0)
    dist = 0.5/sqrt(prox)+ROBOT_RADIUS;
  else
    dist = RANGE_MAX;
  if (dist > RANGE_MAX)
    dist = RANGE_MAX;
  return dist;
}

float to_radians(float degrees) {
  return degrees * M_PI / 180.0;
}

float to_degrees(float radians) {
  return radians * 180.0 / M_PI;
}

float normalize_angle(float angle) {
  const float result = fmod(angle, 2.0*M_PI);
  if(result < 0.0) 
    return result + 2.0*M_PI;
  return result;
}
  
int main(int argc, char **argv) {
  FILE *log = fopen("log.csv", "w");
  if (!log)
    return 1;
    
  ////// EXERCICIO: CRIAR FUNÇÃO PARA SUBSTITUIR ESSE CODIGO USANDO STRUCT /////
  wb_robot_init();
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.1 * MAX_SPEED);
  wb_motor_set_velocity(right_motor, 0.1 * MAX_SPEED);
  WbDeviceTag left_encoder = wb_robot_get_device("left wheel sensor");
  WbDeviceTag right_encoder = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_encoder, TIME_STEP);
  wb_position_sensor_enable(right_encoder, TIME_STEP);
  WbDeviceTag ps[8];
  char ps_id[4];
  for(int i = 0; i < 8; i++){
    sprintf(ps_id, "ps%d", i);
    ps[i] = wb_robot_get_device(ps_id);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  ////// EXERCICIO: CRIAR FUNÇÃO PARA SUBSTITUIR ESSE CODIGO USANDO STRUCT /////

  float dist[8];
  float x = 0, y = 0, theta = 0;
  float left_steps_prev = 0, right_steps_prev = 0;
  while (wb_robot_step(TIME_STEP) != -1) {
    // CALCULAR POSIÇÃO DO ROBO.
    float left_steps = wb_position_sensor_get_value(left_encoder);
    float right_steps = wb_position_sensor_get_value(right_encoder);
    float left_steps_diff = left_steps * WHEEL_RADIUS - left_steps_prev; // Expressed in meters.
    float right_steps_diff = right_steps * WHEEL_RADIUS - right_steps_prev;   // Expressed in meters.
    float delta_theta = (right_steps_diff - left_steps_diff)/WHEEL_DISTANCE;   // Expressed in radians.
    float delta_steps = (right_steps_diff + left_steps_diff)/2.0;        // Expressed in meters.
    x += delta_steps * cos(theta + delta_theta/2.0);   // Expressed in meters.
    y += delta_steps * sin(theta + delta_theta/2.0);   // Expressed in meters.
    theta += delta_theta;    // Expressed in radians.
    theta = normalize_angle(theta);
    left_steps_prev = left_steps * WHEEL_RADIUS;     // Expressed in meters.
    right_steps_prev = right_steps * WHEEL_RADIUS;    // Expressed in meters.
    // CALCULAR POSIÇÃO DO ROBO.

    // SALVAR ARQUIVO CONTENDO EM CADA LINHA A POSIÇÃO DO ROBÔ E AS DISTÂNCIAS PARA OS OBSTÁCULOS
    fprintf(log, "%f %f %f ", x, y, theta);
    for(int i = 0; i < 8; i++) {
      dist[i] = convert_intensity_to_meters(wb_distance_sensor_get_value(ps[i]));
      fprintf(log, "%f ", dist[i]);
    }
    fprintf(log, "\n");
    fflush(log);
    // SALVAR ARQUIVO CONTENDO EM CADA LINHA A POSIÇÃO DO ROBÔ E AS DISTÂNCIAS PARA OS OBSTÁCULOS
    
    if ( detect_obstacle_ahead(dist) )
    {
      wb_motor_set_velocity(left_motor, 0.2 * MAX_SPEED);
      wb_motor_set_velocity(right_motor, -0.2 * MAX_SPEED);
    }
    if ( !detect_obstacle_ahead(dist) )
    {
      wb_motor_set_velocity(left_motor, 0.5 * MAX_SPEED);
      wb_motor_set_velocity(right_motor, 0.5 * MAX_SPEED);
    }
  }
   
  fclose(log);

  wb_robot_cleanup();

  return 0;
}
