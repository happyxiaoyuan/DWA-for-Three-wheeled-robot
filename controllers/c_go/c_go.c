
#include <webots/device.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/nodes.h>
#include <webots/robot.h>


static WbDeviceTag left_motor, right_motor;

static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}

static void step() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void init_devices() {
  left_motor = wb_robot_get_device("left_motor ");
  right_motor = wb_robot_get_device("right_motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  step();
}


static void set_actuators() {
  wb_motor_set_velocity(left_motor, 6.0);
  wb_motor_set_velocity(right_motor, 6.0);
}



int main(int argc, char **argv) {
  wb_robot_init();
  init_devices();
  while (true) {
    set_actuators();
    step();
  };

  return EXIT_SUCCESS;
}