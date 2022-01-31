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


// Set the drone's hovering height to 1.2m
const float currentZ = 1.2;
TaskHandle_t xSpinHandle;
// Set the x axis (forward) velocity to 0
float newVx = 0;
// Create a "setpoint" variable
static setpoint_t setpoint;

// The following function is in-charge of rotating the drone towards the source
static void spinTask(void *var)
{
	DEBUG_PRINT("spinTask initiated\n");

	// Initialize yaw rotation variables
	volatile float newYaw = 0;
	float relYaw = 0;
	
	// If not using "stationary" mode, x axis velocity will be set
	// to 0.3m/s next time a setpoint is set
	if (!STATIONARY)
		newVx = 0.3;

	// Initialize the angle variable
	uint8_t angle[] = "000\n\r";

	// Quickly blink the LEDs, so we could see the task has started
	for (int i = 0; i < 3; i++)
	{
		ledSet(3, 1);
		vTaskDelay(M2T(50));
		ledSet(3, 0);
		vTaskDelay(M2T(50));
	}

	ledClearAll();

	// Infinite loop
	while (1)
	{
		DEBUG_PRINT("spinTask Loop\n");
		char *pAngle = (char *)angle;

		// Read the first char from UART
		uart1Getchar(pAngle);
		// As long as the UART message is not over
		while( (*pAngle) != '\r')
		{
			// Read the next char from UART
			uart1Getchar(++pAngle);
		}

		// Finished reading UART, verify message is valid rotation (-90 - 90 degrees)
		if (((angle[0] == '0') || (angle[0] == '1')) ||
			((angle[1] >= '0') || (angle[1] <= '9')) ||
			((angle[2] >= '0') || (angle[2] <= '9')))
		{
			
			// Valid angle, parse the message
			relYaw = (angle[0] == 48U) - (angle[0] == 49U);
			relYaw = relYaw * ((angle[1] - 48U) * 10 + (angle[2] - 48U));
			newYaw += relYaw;
			// The drone will rotate by newYaw degrees when next setpoint is set
			newYaw = fmodf(newYaw, 360);

		}

		// Set a new setpoint, applying all changes
		setHoverSetpoint(&setpoint, newVx, 0, currentZ, newYaw);

		vTaskDelay(M2T(250));
	}
}


// A function setting a new setpoint with the given arguments
// only planning the movement, not executing yet.
// go to height z, rotate by yaw, move with velocity vector (vx, vy)
static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yaw)
{
	
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  setpoint->mode.yaw = modeAbs;
  setpoint->attitude.yaw = yaw;
  setpoint->mode.x = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.y = vy;
  setpoint->velocity_body = TRUE;
}

static void __attribute__((used)) hoverTask(void *var)
{
	systemWaitStart();
	vTaskDelay(M2T(3000));
	DEBUG_PRINT("Initialized\n");

	 // Blink the LEDs for stabilization notification ("finished stabilization")
	for (int i = 0; i < 3; i++)
		{
			ledSet(3, 1);
			vTaskDelay(M2T(100));
			ledSet(3, 0);
			vTaskDelay(M2T(100));
		}

	// Gradual vertical take-off to 1.2m
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

	// Creating the spinTask task, starting to listen to UART messages
	xTaskCreate(spinTask, "spin", 2 * 150, NULL,
	                /*priority*/4 , NULL);
	DEBUG_PRINT("spinTask Created\n");
	
	// Entering infinite loop
	while (1)
	{
		taskENTER_CRITICAL();
		// Setpoint being set at commander
		// Executing all planned movement
		commanderSetSetpoint(&setpoint, 3);
		taskEXIT_CRITICAL();
		vTaskDelay(M2T(150));
	}
}

// Sequence initialization - OS will execute when powering on the drone
// Creating he hoverTask task, beginning to hover at place
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
