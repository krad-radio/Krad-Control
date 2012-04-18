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

#include <SDL/SDL.h>

typedef struct krad_transmitter_St krad_transmitter_t;

struct krad_transmitter_St {

	int sd;
	unsigned char *data;
	int port;
	char *ip;

};

krad_transmitter_t *krad_transmitter_create (char *ip, int port) {

	krad_transmitter_t *krad_transmitter = calloc(1, sizeof(krad_transmitter_t));

	krad_transmitter->data = calloc(1, 2048);
	strcpy ((char *)krad_transmitter->data, "KC");

	krad_transmitter->ip = strdup (ip);
	krad_transmitter->port = port;

	krad_transmitter->sd = socket (AF_INET, SOCK_DGRAM, 0);

	return krad_transmitter;

}

void krad_transmitter_destroy (krad_transmitter_t *krad_transmitter) {
	
	close (krad_transmitter->sd);
	free (krad_transmitter->ip);
	free (krad_transmitter->data);
	free (krad_transmitter);
}

void krad_control_transmitter_sendto (krad_transmitter_t *krad_transmitter, unsigned char *data, int size) {

	int ret;

	int packet_size;
	struct sockaddr_in remote_client;
		
	ret = 0;
	packet_size = 0;
	
	memset((char *) &remote_client, 0, sizeof(remote_client));
	remote_client.sin_port = htons(krad_transmitter->port);
	remote_client.sin_family = AF_INET;
	
	if (inet_pton(remote_client.sin_family, krad_transmitter->ip, &(remote_client.sin_addr)) != 1) {
		fprintf(stderr, "inet_pton() failed\n");
		exit(1);
	}

	packet_size = size + 2;
	
	memcpy (krad_transmitter->data + 2, data, size);

	ret = sendto(krad_transmitter->sd, krad_transmitter->data, packet_size, 0, 
				 (struct sockaddr *) &remote_client, sizeof(remote_client));
	
	if (ret != packet_size) {
		printf("udp sending error\n");
		exit(1);
	}

}

int krad_transmitter_run (krad_transmitter_t *krad_transmitter) {

	double curr_pos;
	double min_pos, max_pos;
	float middle_pos, pos_range;
	SDL_Event evt;
	SDL_Joystick *joystick;
	int num_axes, num_buttons, num_balls, num_hats;
	int endit;
	float x, y;

	int cmdlen;
	char command[256];

	cmdlen = 0;

	endit = 0;

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

	num_axes = SDL_JoystickNumAxes(joystick);
	num_buttons = SDL_JoystickNumButtons(joystick);
	num_balls = SDL_JoystickNumBalls(joystick);
	num_hats = SDL_JoystickNumHats(joystick);

	printf("%s: %d axes, %d buttons, %d balls, %d hats\n", SDL_JoystickName(0), num_axes, num_buttons, num_balls, num_hats);

	middle_pos = (min_pos + max_pos) / 2.0;
	pos_range = max_pos - middle_pos;
	curr_pos = middle_pos;
	
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
				//printf("%d %d", evt.jaxis.axis, evt.jaxis.value);
				if (evt.jaxis.axis == 0) {
					x = evt.jaxis.value;
					if (x > 0) {
						curr_pos = middle_pos + (((x / 32767.0) * 1.0) * pos_range);					
						//printf(" -- %f %f \n", ((x / 32767.0) * 100.0), curr_pos);

					}
					if (x < 0) {
						curr_pos = middle_pos - (((x / -32767.0) * 1.0) * pos_range);
						//printf(" -- %f %f \n", ((x / -32767.0) * 100.0), curr_pos);

					}					
					
					if ((x == 0) || ((x < 220) && (x > -220))) {
						curr_pos = middle_pos;
					}
					
					//CPhidgetAdvancedServo_setPosition (servo_controller, servo, curr_pos);
					
					cmdlen = sprintf(command, "%d,%f", time(NULL), curr_pos);
					krad_control_transmitter_sendto (krad_transmitter, (unsigned char *)command, cmdlen);
					
					printf ("sending %s\n", command);
					
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

	//CPhidgetAdvancedServo_setEngaged(servo_controller, servo, 0);
	//since user input has been read, this is a signal to terminate the program so we will close the phidget and delete the object we created
	printf("Closing...\n");
	//CPhidget_close((CPhidgetHandle)servo_controller);
	//CPhidget_delete((CPhidgetHandle)servo_controller);


	SDL_JoystickClose (joystick);

	//all done, exit
	return 0;
}

int main(int argc, char* argv[]) {

	krad_transmitter_t *krad_transmitter;

	if (argc != 3) {
		printf ("%s [ip] [port]", argv[0]);
		exit (1);
	}
	
	krad_transmitter = krad_transmitter_create (argv[1], atoi(argv[2]));

	krad_transmitter_run (krad_transmitter);
	
	krad_transmitter_destroy (krad_transmitter);

	return 0;
}

