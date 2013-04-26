/*
 * File:          swarm_controller.c
 * Date:          
 * Description:   
 * Author:        Andreas Hagen, Odd Andreas Sorsather
 * Modifications: 
 */

#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/led.h>
#include "search.c"
#include "stagnation.c"
#include "retrieval.c"
#include <stdlib.h>
#include <stdio.h>

#define TIME_STEP 64
#define EPOCH_CAP 40
#define SEARCH_THRESH 250
#define SEARCH_LAYER = 0
#define STAGNATION_LAYER = 1
#define RETRIEVAL_LAYER = 2
#define TRUE 1
#define FALSE 0
#define DIST_THRESHOLD 300
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */


int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
   
  /*
  * Most of this initialization code we found at http://www.cyberbotics.com/dvd/common/doc/webots/guide/section7.5.html
  * Changed some of the names to make them more intiutive for ourselves.
  *
  */

  // initialize devices
  WbDeviceTag ps[8];
  char ps_names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };
  
  
  for (i=0; i<8 ; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  
  WbDeviceTag ls[8];
  char ls_names[8][4] = {
    "ls0", "ls1", "ls2", "ls3",
    "ls4", "ls5", "ls6", "ls7"
  };
  
  for (i=0; i<8 ; i++) {
    ls[i] = wb_robot_get_device(ls_names[i]);
    wb_light_sensor_enable(ls[i], TIME_STEP);
  }
  
  WbDeviceTag led[8];
  char led_names[8][5] = {
    "led0", "led1", "led2", "led3",
    "led4", "led5", "led6", "led7", "led8"
  };
  

  for (i=0; i<8 ; i++) {
    led[i] = wb_robot_get_device(led_names[i]);
  }

  // Variable declerations
  double distance_sensors[8];
  int light_sensors[8];
  double retrieval_left_wheel_speed;
  double retrieval_right_wheel_speed; 
  double search_left_wheel_speed;
  double search_right_wheel_speed;
  double stagnation_left_wheel_speed;
  double stagnation_right_wheel_speed;
  int i;
  double previous_distance_sensors[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  int EPOCH_TIME = 0; // # TimeSteps the robot tries to push, ranges from 100 to 300
  int STOP = 0;  
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  
  while (wb_robot_step(TIME_STEP) != -1) {
    
	  int CONTROLLING_LAYER = 0;
      int stagnation = get_stagnation_state();
	  //Reading the sensors
		for(i=0; i<8; i++){
			distance_sensors[i] = wb_distance_sensor_get_value(ps[i]);
		}

		 for (i=0; i<8 ; i++){
			light_sensors[i] = wb_light_sensor_get_value(ls[i]);
		}
	
	//int stagnation = stagnation_recovery(distance_sensors, DIST_THRESHOLD);
	
	//Sending relevant sensory input to the search layer, retrieves proposed action
    update_search_speed(distance_sensors, SEARCH_THRESH);
    search_left_wheel_speed = get_search_left_wheel_speed();
    search_right_wheel_speed = get_search_right_wheel_speed();

	//Sending relevant sensory input to the search layer, retrieves proposed action

	int senses_something = swarm_retrieval(light_sensors, RETRIEVAL_THRESH);

    if(senses_something){
        
        CONTROLLING_LAYER = 1;
    }

    retrieval_left_wheel_speed = get_retrieval_left_wheel_speed();		
    retrieval_right_wheel_speed = get_retrieval_right_wheel_speed();

    if(stagnation == 0 && distance_sensors[7] > DIST_THRESHOLD){ 
        EPOCH_TIME = EPOCH_TIME+1;
       if(EPOCH_TIME == EPOCH_CAP - 10){
           for(i = 0; i < 8; i++){
              previous_distance_sensors[i]=distance_sensors[i];
              STOP = 1;
           }          
       }
       else if(EPOCH_TIME > EPOCH_CAP){ 
          EPOCH_TIME = 0;
          STOP = 0;
          reset_stagnation();
          valuate_pushing(distance_sensors,previous_distance_sensors);
       }
    }
    else STOP = 0;


if(stagnation){
CONTROLLING_LAYER=2;
stagnation_recovery(distance_sensors, DIST_THRESHOLD);
stagnation_left_wheel_speed = get_stagnation_left_wheel_speed();
stagnation_right_wheel_speed = get_stagnation_right_wheel_speed();
}
            if(STOP == 1) wb_differential_wheels_set_speed(0,0);
	else if(CONTROLLING_LAYER==1){
	     wb_differential_wheels_set_speed(retrieval_left_wheel_speed,retrieval_right_wheel_speed);
               
	}else if(CONTROLLING_LAYER==2){    
         wb_differential_wheels_set_speed(stagnation_left_wheel_speed,stagnation_right_wheel_speed);

	}else{
		wb_differential_wheels_set_speed(search_left_wheel_speed,search_right_wheel_speed);
	}
  }
  wb_robot_cleanup();
  return 0;
}