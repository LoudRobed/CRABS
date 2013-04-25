/*
 * File:          swarm_controller.c
 * Date:          
 * Description:   
 * Author:        Andreas Hagen, Odd Andreas Sorsather
 * Modifications: 
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/led.h>
#include "search.c"
#include "stagnation.c"
#include "retrieval.c"
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define SEARCH_THRESH 250
#define RETRIEVAL_THRESH 2300
#define SEARCH_LAYER = 0
#define STAGNATION_LAYER = 1
#define RETRIEVAL_LAYER = 2
#define TRUE 1
#define FALSE 0
#define DIST_THRESHOLD 400
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
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
   
  /*
  * Most of this initialization code we found at http://www.cyberbotics.com/dvd/common/doc/webots/guide/section7.5.html
  * Changed some of the names to make them more intiutive for ourselves.
  *
  */
  
  WbDeviceTag ps[8];
  char ps_names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };
  
  // initialize devices
  for (i=0; i<8 ; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  
  WbDeviceTag ls[8];
  char ls_names[8][4] = {
    "ls0", "ls1", "ls2", "ls3",
    "ls4", "ls5", "ls6", "ls7"
  };
  
  // initialize devices
  for (i=0; i<8 ; i++) {
    ls[i] = wb_robot_get_device(ls_names[i]);
    wb_light_sensor_enable(ls[i], TIME_STEP);
  }
  
  WbDeviceTag led[8];
  char led_names[8][5] = {
    "led0", "led1", "led2", "led3",
    "led4", "led5", "led6", "led7"
  };
  
  // initialize devices
  for (i=0; i<8 ; i++) {
    led[i] = wb_robot_get_device(led_names[i]);
  }
  
  int counter = 0;
  double previous_distance_sensors[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
   double distance_sensors[8];
      int light_sensors[8];
      double retrieval_left_wheel_speed;
      double retrieval_right_wheel_speed; 
      double search_left_wheel_speed;
      double search_right_wheel_speed;
      double stagnation_left_wheel_speed;
      double stagnation_right_wheel_speed;
      int i;
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  int EPOCH_TIME = 150; // # TimeSteps the robot tries to push, ranges from 100 to 300
  int STOP_TIME = 0; // # Counts the TimeSteps the robot stands still before evaluating weather to push or realign.
  int NOF_REALIGNMENTS = 0;
  while (wb_robot_step(TIME_STEP) != -1) {
      int CONTROLLING_LAYER = 0;
      

	//Getting data from the search layer
	int stagnation = get_stagnation_state();
	printf("Stagnation: %d \n", stagnation);
	for(i=0; i<8; i++){
		distance_sensors[i] = wb_distance_sensor_get_value(ps[i]);
	}

    update_search_speed(distance_sensors, SEARCH_THRESH);
    search_left_wheel_speed = get_search_left_wheel_speed();
    search_right_wheel_speed = get_search_right_wheel_speed();

    //Getting data from the retrieval layer:
    
    for (i=0; i<8 ; i++){
       light_sensors[i] = wb_light_sensor_get_value(ls[i]);
    }
    int senses_something = swarm_retrieval(light_sensors, RETRIEVAL_THRESH);

    if(senses_something){
        CONTROLLING_LAYER = 1;
    }

    retrieval_left_wheel_speed = get_retrieval_left_wheel_speed();		
    retrieval_right_wheel_speed = get_retrieval_right_wheel_speed();

	// Pseudo-code for stagnation:
	// Check if robot is pushing
	// Check if it has been pushing for some time
	// Try re-aligning with the box
	// If re-aligning has been tried a given number of times, find a new spot.
    
    if(stagnation == 0 && distance_sensors[7] > 300){ //Robot is simply pushing along, not a care in the world.
//printf("No stagnation?");
        EPOCH_TIME = EPOCH_TIME+1;

       if(EPOCH_TIME > 150){ //Robot has been pushing for 150 (billion years), lets see how it's doing!
          EPOCH_TIME = 0;
          //This function evaluates how the pushing is going and sets the stagnation flag.
          reset_stagnation();
          valuate_pushing(distance_sensors,previous_distance_sensors);
       }
    }
    for(i = 0; i < 8; i++){
              previous_distance_sensors[i]=distance_sensors[i];
     }
if(stagnation){
CONTROLLING_LAYER=2;
stagnation_recovery(distance_sensors, DIST_THRESHOLD);
stagnation_left_wheel_speed = get_stagnation_left_wheel_speed();
stagnation_right_wheel_speed = get_stagnation_right_wheel_speed();
}
	if(CONTROLLING_LAYER==1){
	     wb_differential_wheels_set_speed(retrieval_left_wheel_speed,retrieval_right_wheel_speed);
               
	}else if(CONTROLLING_LAYER==2){    

               wb_differential_wheels_set_speed(stagnation_left_wheel_speed,stagnation_right_wheel_speed);

	}else{
		wb_differential_wheels_set_speed(search_left_wheel_speed,search_right_wheel_speed);
	}
  }
  wb_robot_cleanup();
}