#include "deck.h"
#include "system.h"
#include "commander.h"

#ifndef FALSE
#define FALSE 0
#define TRUE 1
#endif

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

#include "debug.h"
#include "uart1.h"
#include "led.h"
#include "hover.h"

#define DEBUG_MODULE "SEQ"

#define STATIONARY 1

const float currentZ = 1.2;
TaskHandle_t xSpinHandle;
float newVx = 0;

static setpoint_t setpoint;

static void spinTask(void *var)
{
	DEBUG_PRINT("spinTask initiated\n");

	volatile float newYaw = 0;
	float relYaw = 0;
	if (!STATIONARY)
		newVx = 0.3;

	uint8_t angle[] = "000\n\r";

//	systemWaitStart(); // Moved to hoverTask
//	vTaskDelay(M2T(2000)); // Moved to hoverTask

	for (int i = 0; i < 3; i++)
	{
		ledSet(3, 1);
		vTaskDelay(M2T(50));
		ledSet(3, 0);
		vTaskDelay(M2T(50));
	}

	ledClearAll();

	while (1)
	{
		DEBUG_PRINT("spinTask Loop\n");
		char *pAngle = (char *)angle;

		uart1Getchar(pAngle);

		while( (*pAngle) != '\r')
		{
//			DEBUG_PRINT("Reading UART loop\n");
//			ledSet(2, 1);
			uart1Getchar(++pAngle);
//			ledSet(2, 0);
		}

		//DEBUG_PRINT("Finished reading UART\n");
		if (((angle[0] == '0') || (angle[0] == '1')) ||
			((angle[1] >= '0') || (angle[1] <= '9')) ||
			((angle[2] >= '0') || (angle[2] <= '9')))
		{
			//DEBUG_PRINT("Valid angle\n");

			relYaw = (angle[0] == 48U) - (angle[0] == 49U);
			relYaw = relYaw * ((angle[1] - 48U) * 10 + (angle[2] - 48U));
			newYaw += relYaw;
			newYaw = fmodf(newYaw, 360);

			// Commented the below for static rotation experiment
			//newVx = 0.3;

			// Added the below for static rotation experiment
//			newVx = 0;
		}

		setHoverSetpoint(&setpoint, newVx, 0, currentZ, newYaw);
		//DEBUG_PRINT("New setpoint was set\n");

		vTaskDelay(M2T(250));
	}
}


static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yaw)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;


  //setpoint->mode.yaw = modeVelocity;
  setpoint->mode.yaw = modeAbs;
  //setpoint->attitudeRate.yaw = yaw;
  setpoint->attitude.yaw = yaw;

  setpoint->mode.x = modeVelocity;
  setpoint->velocity.x = vx;

  setpoint->mode.y = modeVelocity;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = TRUE;
}

static void __attribute__((used)) hoverTask(void *var)
{
	systemWaitStart(); // Moved from spinTask
	vTaskDelay(M2T(3000)); // Moved from spinTask
	DEBUG_PRINT("Initialized\n");

	 // Added for stabilization notification
	for (int i = 0; i < 3; i++)
		{
			ledSet(3, 1);
			vTaskDelay(M2T(100));
			ledSet(3, 0);
			vTaskDelay(M2T(100));
		}

	// Added for gradual take-off
	for (int i = 1; i <= 24; i++)
		{
			setHoverSetpoint(&setpoint, 0, 0, i*0.05, 0);
			taskENTER_CRITICAL();
			commanderSetSetpoint(&setpoint, 3);
			taskEXIT_CRITICAL();
			vTaskDelay(M2T(200));
		}

	taskENTER_CRITICAL();
	commanderSetSetpoint(&setpoint, 3);
	taskEXIT_CRITICAL();
	vTaskDelay(M2T(150));

	// Added for gradual take-off
	//setHoverSetpoint(&setpoint, 0, 0, 0.3, 0);
	//commanderSetSetpoint(&setpoint, 3);
	//vTaskDelay(M2T(300));

	// Added for hovering before UART inputs can be received
//	setHoverSetpoint(&setpoint, 0, 0, currentZ, 0);
//	commanderSetSetpoint(&setpoint, 3);
//	vTaskDelay(M2T(300));
//	commanderSetSetpoint(&setpoint, 3);
//	vTaskDelay(M2T(300));

	xTaskCreate(spinTask, "spin", 2 * 150, NULL,
	                /*priority*/4 , NULL);
	DEBUG_PRINT("spinTask Created\n");
	while (1)
	{
		taskENTER_CRITICAL();
		commanderSetSetpoint(&setpoint, 3);
		taskEXIT_CRITICAL();
//		DEBUG_PRINT("Hover setpoint was set\n");
		vTaskDelay(M2T(150));
	}
}


static void sequenceInit()
{
    xTaskCreate(hoverTask, "hover", 2 * 150, NULL,
            /*priority*/3, NULL);
    DEBUG_PRINT("hoverTask created\n");
}

static bool sequenceTest()
{
  return 1;
}

const DeckDriver sequence_deck = {
  .vid = 0,
  .pid = 0,
  .name = "roeiHover",

  .usedGpio = DECK_USING_UART1 ,

  .init = sequenceInit,
  .test = sequenceTest,
};

DECK_DRIVER(sequence_deck);
