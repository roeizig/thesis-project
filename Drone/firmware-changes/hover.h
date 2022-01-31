/*
 * hover.h
 *
 *  Created on: Jun 6, 2019
 *      Author: Roei
 */

#ifndef SRC_DECK_DRIVERS_SRC_HOVER_H_
#define SRC_DECK_DRIVERS_SRC_HOVER_H_

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate);
static void spinTask(void *var);
static void hoverTask(void *var);

#endif /* SRC_DECK_DRIVERS_SRC_HOVER_H_ */
