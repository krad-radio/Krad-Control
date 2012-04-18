#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <malloc.h>
#include <time.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sched.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <errno.h>
#include <sys/socket.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <poll.h>

#include <phidget21.h>



typedef struct krad_receiver_St krad_receiver_t;

struct krad_receiver_St {

	int sd;
	unsigned char *data;
	int port;
	struct sockaddr_in local_address;
	struct sockaddr_in remote_address;
};

krad_receiver_t *krad_receiver_create (int port) {

	krad_receiver_t *krad_receiver = calloc(1, sizeof(krad_receiver_t));

	krad_receiver->data = calloc(1, 2048);
	strcpy ((char *)krad_receiver->data, "KC");

	krad_receiver->port = port;

	krad_receiver->sd = socket (AF_INET, SOCK_DGRAM, 0);

	memset((char *) &krad_receiver->local_address, 0, sizeof(krad_receiver->local_address));
	krad_receiver->local_address.sin_family = AF_INET;
	krad_receiver->local_address.sin_port = htons (krad_receiver->port);
	krad_receiver->local_address.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind (krad_receiver->sd, (struct sockaddr *)&krad_receiver->local_address, sizeof(krad_receiver->local_address)) == -1 ) {
		printf("bind error\n");
		exit(1);
	}

	return krad_receiver;

}

void krad_receiver_destroy (krad_receiver_t *krad_receiver) {
	
	close (krad_receiver->sd);

	free (krad_receiver->data);
	free (krad_receiver);
}


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
//	printf("Motor: %d > Current Position: %f\n", Index, Value);
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

int krad_receiver_run (krad_receiver_t *krad_receiver) {

	int rsize;
	int result;
	int ret;
	double curr_pos;
	const char *err;
	double Accel, maxVel;
	int servo;	
	double min_pos, max_pos;
	float middle_pos, pos_range;
	int num_axes, num_buttons, num_balls, num_hats;
	int endit;
	float x, y;
	float val;
	CPhidgetAdvancedServoHandle servo_controller;

	//Declare a motor control handle
	CPhidgetMotorControlHandle motoControl = 0;

	//create the motor control object
	CPhidgetMotorControl_create(&motoControl);
	CPhidget_open((CPhidgetHandle)motoControl, -1);
	printf("Waiting for MotorControl to be attached....");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)motoControl, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return 0;
	}


	servo_controller = 0;
	endit = 0;
	servo = 0;
	val = 0;

	min_pos = 76.0;
	max_pos = 149.0;


	//servo = 7;
	//min_pos = 0.0;
	//max_pos = 233.0;

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
	
	CPhidgetMotorControl_setAcceleration (motoControl, 0, 10.00);	
	
	while (1) {
	
		ret = recvfrom(krad_receiver->sd, krad_receiver->data, 2000, 0, (struct sockaddr *)&krad_receiver->remote_address, (socklen_t *)&rsize);
		
		if (ret == -1) {
			printf("failed recvin udp\n");
			return 1;
		}
		
		printf ("got packet! %s\n", krad_receiver->data);		
		
		val = atof (krad_receiver->data[3]);
		
		if (krad_receiver->data[2] == 'S') {
			CPhidgetAdvancedServo_setPosition (servo_controller, servo, val);
		}
		
		if (krad_receiver->data[2] == 'T') {
		
			CPhidgetMotorControl_setVelocity (motoControl, 0, val);	
			CPhidgetMotorControl_setVelocity (motoControl, 1, val);		
		
		}

	}

	printf("endit is %d\n", endit);

	CPhidgetAdvancedServo_setEngaged(servo_controller, servo, 0);
	//since user input has been read, this is a signal to terminate the program so we will close the phidget and delete the object we created
	printf("Closing servo...\n");
	CPhidget_close((CPhidgetHandle)servo_controller);
	CPhidget_delete((CPhidgetHandle)servo_controller);

	printf("Closing motor...\n");
	CPhidget_close((CPhidgetHandle)motoControl);
	CPhidget_delete((CPhidgetHandle)motoControl);

	//all done, exit
	return 0;
}

int main(int argc, char* argv[]) {

	krad_receiver_t *krad_receiver;

	if (argc != 2) {
		printf ("%s [port]", argv[0]);
		exit (1);
	}
	
	krad_receiver = krad_receiver_create (atoi(argv[1]));

	krad_receiver_run (krad_receiver);
	
	krad_receiver_destroy (krad_receiver);

	return 0;
}

