#include <math.h>
#include <stdio.h>
#include "controller.h"
#include "kobukiSensorTypes.h"
#include "display.h"

// configure initial state
KobukiSensors_t sensors = {0};
uint16_t previous_encoder = 0;
float distance = 0;

bool line_is_right;
bool line_is_center;
bool line_is_left;

float Kp = 0.1;
float Ki = 0.0008;
float Kd = .6;

int P;
int I;
int D;

int lastError = 0;

uint16_t basespeedL = 50;
uint16_t basespeedR = 50;

const uint16_t maxspeedL = 150;
const uint16_t maxspeedR = 150;

static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder) {
  const float CONVERSION = 0.00065;

  float result = 0.0;
  if (current_encoder >= previous_encoder) {
    result = (float)current_encoder - (float)previous_encoder;
  } else {
    result = (float)current_encoder + (0xFFFF - (float)previous_encoder);
  }
  return result = result * CONVERSION;
}

uint32_t mean(uint32_t* arr, uint32_t arr_size) {
  uint32_t sum = 0;
  for (uint32_t i = 0; i < arr_size; i++) {
    sum += arr[i];
  }
  return sum / arr_size;
}

static void check_line(KobukiSensors_t* sensors, bool* line_is_right, bool* line_is_left, bool* line_is_center) {
  // Your code here
	*line_is_right = false;
	*line_is_left = false;
	*line_is_center = false;

	if (sensors->cliffLeft) {
		*line_is_left = true;
	}
	if (sensors->cliffRight) {
		*line_is_right = true;
	}
	if (sensors->cliffCenter) {
		*line_is_center = true;
	}
}


robot_state_t controller(robot_state_t state, uint32_t* dist_mm_ptr, uint32_t num_measurements, uint32_t* data, uint8_t* instruction) {
  // read sensors from robot
  kobukiSensorPoll(&sensors);

  // delay before continuing
  // Note: removing this delay will make responses quicker, but will result
  //  in printf's in this loop breaking JTAG
  //nrf_delay_ms(1);

  // char buf[16];
  // snprintf(buf, 16, "%f", distance);
  // display_write(buf, DISPLAY_LINE_1);

  check_line(&sensors, &line_is_right, &line_is_left, &line_is_center);

  // handle states
  switch(state) {
    case OFF: {
      // transition logic
      if (is_button_pressed(&sensors)) {
        state = DRIVING;
        previous_encoder = sensors.leftWheelEncoder;
      } else {
        // perform state-specific actions here
        //display_write("OFF", DISPLAY_LINE_0);

        if (line_is_right && line_is_left) {
        			display_write("RIGHT AND LEFT", DISPLAY_LINE_0);
                } else if (line_is_left) {
                	display_write("LEFT", DISPLAY_LINE_0);
                } else if (line_is_right) {
                	display_write("RIGHT", DISPLAY_LINE_0);
                } else {
                	display_write("NONE", DISPLAY_LINE_0);
                }

                if (line_is_center) {
                	display_write("CENTER", DISPLAY_LINE_1);
                } else {
                	display_write("NOT CENTER", DISPLAY_LINE_1);
                }

        kobukiDriveDirect(0,0);
        state = OFF;
      }
      break; // each case needs to end with break!
    }

    case DRIVING: {
      // transition logic
      if (is_button_pressed(&sensors)) {
        state = OFF;
      } else {
        // perform state-specific actions here
        uint32_t mean_dist = mean(dist_mm_ptr, num_measurements);
        *(data+2) = mean_dist;
        printf("mean dist: %d\n", mean_dist);
        //ultrasonic_distance_control(mean_dist);
        if (line_is_right && line_is_left && line_is_center) {
          *(data) = read_timer();
          *(data + 3) += 1;

        }
        basespeedL = (uint8_t)(*(instruction + 1));
        basespeedR = (uint8_t)(*(instruction + 1));

        PID_control();
        state = DRIVING;
      }
      break; // each case needs to end with break!
    }
  }

  // add other cases here
  return state;
}

void ultrasonic_distance_control(uint32_t dist_mm) {
  // maintain 50 mm dist between robots
  // if (dist_mm > 300) {
  //   basespeedL = 125;
  //   basespeedR = 125;
  // } else if (dist_mm < 100) {
  //   basespeedL = 25;
  //   basespeedR = 25;
  // }
  printf("dist_mm: %d\n", dist_mm);
  basespeedL = 50 + .5*(dist_mm - 50);
  basespeedR = 50 + .5* (dist_mm - 50);

  if (basespeedL > 100) {
	    basespeedL = 100;
	  }
	  if (basespeedR > 100) {
	    basespeedR = 100;
	  }
	  if (basespeedL < 25) {
	    basespeedL = 025;
	  }
	  if (basespeedR < 25) {
	    basespeedR = 025;
	  }
}

void PID_control(){
	int position;

	if (line_is_left && !line_is_center && !line_is_right) {
		position = 0;
	} else if (line_is_left && line_is_center && !line_is_right) {
		position = 350/2;
	} else if (!line_is_left && line_is_center && !line_is_right) {
		position = 350;
	} else if (!line_is_left && line_is_center && line_is_right) {
		position = 350 + 350/2;
	} else if (!line_is_left && !line_is_center && line_is_right) {
		position = 700;
	}


	int error = 350 - position;
	P = error;
	I = I + error;
	D = error - lastError;

	lastError = error;

	int motorspeed = P * Kp + I * Ki + D * Kd;

	char buf[16];
	snprintf(buf, 16, "%d", motorspeed);
	display_write(buf, DISPLAY_LINE_1);

	int motorspeedL = basespeedL + motorspeed;
	int motorspeedR = basespeedR - motorspeed;

	if (motorspeedL > maxspeedL) {
	    motorspeedL = maxspeedL;
	  }
	  if (motorspeedR > maxspeedR) {
	    motorspeedR = maxspeedR;
	  }
	  if (motorspeedL < 0) {
	    motorspeedL = 0;
	  }
	  if (motorspeedR < 0) {
	    motorspeedR = 0;
	  }
    printf("motor speed L: %d\n", motorspeedL);
    printf("motor speed R: %d\n", motorspeedR);
    printf("base speed: %d\n", basespeedL);

	  kobukiDriveDirect(motorspeedR,motorspeedL);
}
