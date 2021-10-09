#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/supervisor.h>

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


typedef struct {

WbFieldRef pos;
WbFieldRef rotat;
WbDeviceTag left_motor;
WbDeviceTag right_motor;  
WbDeviceTag ps[8];

}tbotspec;


tbotspec botinit() {


tbotspec spec;

wb_robot_init();
  spec.left_motor = wb_robot_get_device("left wheel motor");
  spec.right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(spec.left_motor, INFINITY);
  wb_motor_set_position(spec.right_motor, INFINITY);
  wb_motor_set_velocity(spec.left_motor, 0.1 * MAX_SPEED);
  wb_motor_set_velocity(spec.right_motor, 0.1 * MAX_SPEED);
  WbDeviceTag left_encoder = wb_robot_get_device("left wheel sensor");
  WbDeviceTag right_encoder = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_encoder, TIME_STEP);
  wb_position_sensor_enable(right_encoder, TIME_STEP);

  char ps_id[4];
  for(int i = 0; i < 8; i++){
    sprintf(ps_id, "ps%d", i);
    spec.ps[i] = wb_robot_get_device(ps_id);
    wb_distance_sensor_enable(spec.ps[i], TIME_STEP);
  }
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("EPUCK");
  if (robot_node == NULL) {
    fprintf(stderr, "No DEF EPUCK node found in the current world file\n");
    exit(1);
  }
  spec.pos = wb_supervisor_node_get_field(robot_node, "translation");
  spec.rotat = wb_supervisor_node_get_field(robot_node, "rotation");


  return spec;
}

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
 
void salvar_posicao_distancias(FILE *log, float x, float y, float angulo, float distancia[8]){
    fprintf(log, "%f %f %f ", x, y, angulo);
    for(int i = 0; i < 8; i++) {
      fprintf(log, "%f ", distancia[i]);
    }
    fprintf(log, "\n");
    fflush(log);
}

int main(int argc, char **argv) {
  FILE *log = fopen("log.csv", "w");
  if (!log)
    exit(1);
    
  ////// EXERCICIO: CRIAR FUNÇÃO PARA SUBSTITUIR ESSE CODIGO USANDO STRUCT /////
 
  ////// EXERCICIO: CRIAR FUNÇÃO PARA SUBSTITUIR ESSE CODIGO USANDO STRUCT /////

  tbotspec configu = botinit();
  
  float dist[8];
  float x = 0, y = 0, theta = 0;
  float left_steps_prev = 0, right_steps_prev = 0;
  while (wb_robot_step(TIME_STEP) != -1) {
    const double *position = wb_supervisor_field_get_sf_vec3f(configu.pos);
    const double *rotation = wb_supervisor_field_get_sf_rotation(configu.rotat);

    for(int i = 0; i < 8; i++) {
      dist[i] = convert_intensity_to_meters(wb_distance_sensor_get_value(configu.ps[i]));
    }

    salvar_posicao_distancias(log, position[0], position[2], rotation[3], dist);

    if ( detect_obstacle_ahead(dist) )
    {
      wb_motor_set_velocity(configu.left_motor, 0.2 * MAX_SPEED);
      wb_motor_set_velocity(configu.right_motor, -0.2 * MAX_SPEED);
    }
    if ( !detect_obstacle_ahead(dist) )
    {
      wb_motor_set_velocity(configu.left_motor, 0.5 * MAX_SPEED);
      wb_motor_set_velocity(configu.right_motor, 0.5 * MAX_SPEED);
    }
  }
   
  fclose(log);

  wb_robot_cleanup();

  return 0;
}
