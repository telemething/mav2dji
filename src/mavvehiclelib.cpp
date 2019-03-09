/*
 * mav2dji_ros.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <mavvehiclelib.hpp>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdbool.h>

#define BUFFER_LENGTH 2041 

//*****************************************************************************
//*
//*
//*
//******************************************************************************

namespace mavvehiclelib 
{

//*****************************************************************************
//*
//*
//*
//******************************************************************************

mavvehicle::mavvehicle()
{
  init();
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

mavvehicle::mavvehicle(mavMessageCallbackType callback)
{
	addMavMessageCallback(callback);
	init();
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

mavvehicle::~mavvehicle()
{
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void mavvehicle::init()
{
		verbose = false;

    vehicleUdpAddress = "";
    vehicleUdpPort = 14551;
    qgcUdpAddress = "127.0.0.1";
    qgcUdpPort = 14550;

		mavlinkSystemId = 1;
		mavlinkComponentId = 1;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void mavvehicle::addMavMessageCallback(mavMessageCallbackType callback)
{
	mavMessageCallback = callback;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

uint64_t mavvehicle::microsSinceEpoch()
{
	struct timeval tv;
	
	uint64_t micros = 0;
	
	gettimeofday(&tv, NULL);  
	micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
	
	return micros;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void mavvehicle::startVehicle()
{
    listenWorkerThreadShouldRun = true;

		int socket1 = createSocket(vehicleUdpAddress, vehicleUdpPort, true);
		//int socket2 = createSocket(vehicleUdpAddress, vehicleUdpPort);

    listenWorkerThread = std::thread(&mavvehicle::listenWorker, this, socket1, qgcUdpAddress, qgcUdpPort);
    sendWorkerThread = std::thread(&mavvehicle::exampleLoop, this, socket1, qgcUdpAddress, qgcUdpPort);
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void mavvehicle::stopVehicle()
{
    listenWorkerThreadShouldRun = false;
}

//*****************************************************************************
//*
//*	Params: address
//*					port : should be 14551
//*
//******************************************************************************

int mavvehicle::createSocket(std::string localAddress, int localPort, bool blocking)
{
	int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	struct sockaddr_in locAddr;
  int option_value = 1;

	if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT | SO_LINGER, &option_value, sizeof(int)) < 0)
    printf("setsockopt(SO_REUSEADDR) failed\n");

	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(vehicleUdpPort);
	
	/* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */ 
	if (-1 == bind(sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr)))
  {
		perror("error bind failed");
		close(sock);
		return -1;
  } 
	
	// Attempt to make it non blocking 
	// we dont need this is we have separate treads for read and write

	if(!blocking)
	{
		if (fcntl(sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
		{
			fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
			close(sock);
			return -1;
		}
	}

	return sock;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void mavvehicle::listenWorker(int sock, std::string fromAddress, int fromPort)
{
	char target_ip[100];
	
	struct sockaddr_in gcAddr; 
	uint8_t buf[BUFFER_LENGTH];
	ssize_t recsize;
	socklen_t fromlen = sizeof(gcAddr);
	mavlink_message_t msg;
	int i = 0;
	unsigned int temp = 0;

	strcpy(target_ip, fromAddress.c_str());

	memset(&gcAddr, 0, sizeof(gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = inet_addr(target_ip);
	gcAddr.sin_port = htons(fromPort);
		
	//for (;;) 
  while(listenWorkerThreadShouldRun)
  {		
		memset(buf, 0, BUFFER_LENGTH);
		recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &fromlen);

		if (recsize > 0)
    {
			// Something received - print out all bytes and parse packet
			mavlink_message_t msg;
			mavlink_status_t status;
			int gcPort = ntohs(gcAddr.sin_port);
			
			if(verbose)
				printf("Bytes Received: %d\nDatagram: ", (int)recsize);

			for (i = 0; i < recsize; ++i)
			{
				temp = buf[i];

				if(verbose)
					printf("%02x ", (unsigned char)temp);

				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
				{
					// Packet received
					if(verbose)
						printf("\nReceived packet: Port: %i, SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", 
							gcPort, msg.sysid, msg.compid, msg.len, msg.msgid);

					mavMessageCallback(msg);
				}
			}

			if(verbose)
				printf("\n");
		}

		memset(buf, 0, BUFFER_LENGTH);
  }
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void mavvehicle::exampleLoop(int sock, std::string toAddress, int toPort)
{
	char target_ip[100];
	
	float position[6] = {};
	struct sockaddr_in gcAddr; 
	struct sockaddr_in fromAddr;
	uint8_t buf[BUFFER_LENGTH];
	ssize_t recsize;
	socklen_t fromlen = sizeof(gcAddr);
	int bytes_sent;
	mavlink_message_t msg;
	uint16_t len;
	int i = 0;
	unsigned int temp = 0;

	strcpy(target_ip, toAddress.c_str());

	memset(&gcAddr, 0, sizeof(gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = inet_addr(target_ip);
	gcAddr.sin_port = htons(toPort);
	
	//for (;;) 
  while(listenWorkerThreadShouldRun)
  {
		
		/*Send Heartbeat */
		mavlink_msg_heartbeat_pack(mavlinkSystemId, mavlinkComponentId, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
		/* Send Status */
		mavlink_msg_sys_status_pack(mavlinkSystemId, mavlinkComponentId, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof (struct sockaddr_in));
		
		/* Send Local Position */
		mavlink_msg_local_position_ned_pack(mavlinkSystemId, mavlinkComponentId, &msg, microsSinceEpoch(), 
										position[0], position[1], position[2],
										position[3], position[4], position[5]);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
		/* Send attitude */
		mavlink_msg_attitude_pack(mavlinkSystemId, mavlinkComponentId, &msg, microsSinceEpoch(), 1.2, 1.7, 3.14, 0.01, 0.02, 0.03);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
		memset(buf, 0, BUFFER_LENGTH);
		sleep(1); // Sleep one second
  }
}

}