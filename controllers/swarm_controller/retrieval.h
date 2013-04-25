/*
 * retrieval.h
 *
 *  Created on: 17. mars 2011
 *      Author: jannik
 */

#ifndef PUSH_H_
#define PUSH_H_

int swarm_retrieval(int IR_sensor_value[8], int IR_threshold);
double get_retrieval_left_wheel_speed();
double get_retrieval_right_wheel_speed();
int get_LED_state(int LED_num);
int pushing_box();
#endif /* PUSH_H_ */
