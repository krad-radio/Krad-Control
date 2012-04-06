#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include <SDL/SDL.h>
#include <ncurses.h>
#include <phidget21.h>

int CCONV AttachHandler(CPhidgetHandle ADVSERVO, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (ADVSERVO, &name);
	CPhidget_getSerialNumber(ADVSERVO, &serialNo);
	printf("%s %10d attached!\n", name, serialNo);
	
	printf("%s %10d attached!\n", name, serialNo);
	
	return 0;
}

int CCONV DetachHandler(CPhidgetHandle ADVSERVO, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (ADVSERVO, &name);
	CPhidget_getSerialNumber(ADVSERVO, &serialNo);
	printf("%s %10d attached!\n", name, serialNo);

	return 0;
}

int CCONV ErrorHandler(CPhidgetHandle ADVSERVO, void *userptr, int ErrorCode, const char *Description)
{

	printf("Error handled. %d - %s\n", ErrorCode, Description);
	return 0;
}

int CCONV PositionChangeHandler(CPhidgetAdvancedServoHandle ADVSERVO, void *usrptr, int Index, double Value)
{
	printf("Motor: %d > Current Position: %f\n", Index, Value);
	return 0;
}

//Display the properties of the attached phidget to the screen.  We will be displaying the name, serial number and version of the attached device.
int display_properties(CPhidgetAdvancedServoHandle phid)
{
	int serialNo, version, numMotors;
	const char* ptr;

	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

	CPhidgetAdvancedServo_getMotorCount (phid, &numMotors);
	
	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n# Motors: %d\n", serialNo, version, numMotors);

	return 0;
}

int servo_simple()
{
	int result;
	double curr_pos;
	const char *err;
	double Accel, maxVel;
	int servo;	
	double min_pos, max_pos;
	float middle_pos, pos_range;
	SDL_Event evt;
	SDL_Joystick *joystick;
	int num_axes, num_buttons, num_balls, num_hats;
	int endit;
	float x, y;
	CPhidgetAdvancedServoHandle servo_controller;

	servo_controller = 0;
	endit = 0;
	servo = 0;

	min_pos = 76.0;
	max_pos = 149.0;


	//servo = 7;
	//min_pos = 0.0;
	//max_pos = 233.0;

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) < 0) {
		fprintf(stderr, "Couldn't initialize SDL: %s\n", SDL_GetError());
		return 1;
	}

	SDL_JoystickEventState(SDL_ENABLE);
	joystick = SDL_JoystickOpen(0);
	assert(joystick);

	num_axes = SDL_JoystickNumAxes(joystick);
	num_buttons = SDL_JoystickNumButtons(joystick);
	num_balls = SDL_JoystickNumBalls(joystick);
	num_hats = SDL_JoystickNumHats(joystick);

	printf("%s: %d axes, %d buttons, %d balls, %d hats\n",
	       SDL_JoystickName(0),
	       num_axes, num_buttons, num_balls, num_hats);

	//create the advanced servo object
	CPhidgetAdvancedServo_create(&servo_controller);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)servo_controller, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)servo_controller, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)servo_controller, ErrorHandler, NULL);

	//Registers a callback that will run when the motor position is changed.
	//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetAdvancedServo_set_OnPositionChange_Handler(servo_controller, PositionChangeHandler, NULL);

	//open the device for connections
	CPhidget_open((CPhidgetHandle)servo_controller, -1);

	//get the program to wait for an advanced servo device to be attached
	printf("Waiting for Phidget to be attached....");
	
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)servo_controller, 10000)))
	{
		//endwin();
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return 0;
	}

	//Display the properties of the attached device
	display_properties(servo_controller);

	//Set up some initial acceleration and velocity values
	CPhidgetAdvancedServo_getAccelerationMax(servo_controller, servo, &Accel);
	CPhidgetAdvancedServo_setAcceleration(servo_controller, servo, Accel - (Accel/3));
	CPhidgetAdvancedServo_getVelocityMax(servo_controller, servo, &maxVel);
	CPhidgetAdvancedServo_setVelocityLimit(servo_controller, servo, maxVel - (maxVel/3));


	middle_pos = (min_pos + max_pos) / 2.0;
	pos_range = max_pos - middle_pos;
	curr_pos = middle_pos;

	CPhidgetAdvancedServo_setPosition (servo_controller, servo, curr_pos);
	CPhidgetAdvancedServo_setEngaged(servo_controller, servo, 1);
	
	while (SDL_WaitEvent(&evt)) {
	
		switch (evt.type) {
			case SDL_KEYDOWN:
                switch ( evt.key.keysym.sym ) {
                    case SDLK_q:
						endit = 1;
                        break;
                    default:
                        break;
                }

			case SDL_QUIT:
				endit = 1;
				break;

			case SDL_JOYAXISMOTION:
				printf("%d %d", evt.jaxis.axis, evt.jaxis.value);
				if (evt.jaxis.axis == 0) {
					x = evt.jaxis.value;
					if (x > 0) {
						curr_pos = middle_pos + (((x / 32767.0) * 1.0) * pos_range);					
						printf(" -- %f %f \n", ((x / 32767.0) * 100.0), curr_pos);

					}
					if (x < 0) {
						curr_pos = middle_pos - (((x / -32767.0) * 1.0) * pos_range);
						printf(" -- %f %f \n", ((x / -32767.0) * 100.0), curr_pos);

					}					
					
					if ((x == 0) || ((x < 220) && (x > -220))) {
						curr_pos = middle_pos;
					}
					
					CPhidgetAdvancedServo_setPosition (servo_controller, servo, curr_pos);
				}
				else if (evt.jaxis.axis == 1) {
					y = evt.jaxis.value;
					y/=1<<15;
				}
				break;
		}

		if (endit) {
			break;
		}
	}

	printf("endit is %d\n", endit);

	CPhidgetAdvancedServo_setEngaged(servo_controller, servo, 0);
	//since user input has been read, this is a signal to terminate the program so we will close the phidget and delete the object we created
	printf("Closing...\n");
	CPhidget_close((CPhidgetHandle)servo_controller);
	CPhidget_delete((CPhidgetHandle)servo_controller);


	SDL_JoystickClose (joystick);

	//all done, exit
	return 0;
}

int main(int argc, char* argv[])
{
	servo_simple();
	return 0;
}

