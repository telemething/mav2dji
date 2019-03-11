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
//#include <arpa/inet.h>
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

		vehicleMavlinkSocket = createSocket(vehicleUdpAddress, vehicleUdpPort, true);
		//int socket2 = createSocket(vehicleUdpAddress, vehicleUdpPort);

    listenWorkerThread = std::thread(&mavvehicle::listenWorker, this, vehicleMavlinkSocket, qgcUdpAddress, qgcUdpPort);
    sendWorkerThread = std::thread(&mavvehicle::exampleLoop, this, vehicleMavlinkSocket, qgcUdpAddress, qgcUdpPort);
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
//*
//*
//******************************************************************************

int mavvehicle::sendMavMessageToGcs(const mavlink_message_t* pMsg)
{
		auto len = mavlink_msg_to_send_buffer(udpSendBuffer, pMsg);

		auto bytes_sent = sendto(vehicleMavlinkSocket, udpSendBuffer, len, 0, 
			(struct sockaddr*)&gcSockAddr, sizeof(struct sockaddr_in));

		memset(udpSendBuffer, 0, BUFFER_LENGTH);
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

	// the vehicle
	memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(vehicleUdpPort);

	// the ground control
	memset(&gcSockAddr, 0, sizeof(gcSockAddr));
	gcSockAddr.sin_family = AF_INET;
	gcSockAddr.sin_addr.s_addr = inet_addr(qgcUdpAddress.c_str());
	gcSockAddr.sin_port = htons(qgcUdpPort);
	
	// Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol 
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
	
	//struct sockaddr_in gcAddr; 
	uint8_t buf[BUFFER_LENGTH];
	ssize_t recsize;
	socklen_t fromlen = sizeof(gcSockAddr);
	mavlink_message_t msg;
	int i = 0;
	unsigned int temp = 0;

	strcpy(target_ip, fromAddress.c_str());

	memset(&gcSockAddr, 0, sizeof(gcSockAddr));
	gcSockAddr.sin_family = AF_INET;
	gcSockAddr.sin_addr.s_addr = inet_addr(target_ip);
	gcSockAddr.sin_port = htons(fromPort);
		
	//for (;;) 
  while(listenWorkerThreadShouldRun)
  {		
		memset(buf, 0, BUFFER_LENGTH);
		recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcSockAddr, &fromlen);

		if (recsize > 0)
    {
			// Something received - print out all bytes and parse packet
			mavlink_message_t msg;
			mavlink_status_t status;
			int gcPort = ntohs(gcSockAddr.sin_port);
			
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
	//struct sockaddr_in gcSockAddr; 
	struct sockaddr_in fromAddr;
	uint8_t buf[BUFFER_LENGTH];
	ssize_t recsize;
	socklen_t fromlen = sizeof(gcSockAddr);
	int bytes_sent;
	mavlink_message_t msg;
	uint16_t len;
	int i = 0;
	unsigned int temp = 0;

	strcpy(target_ip, toAddress.c_str());

	memset(&gcSockAddr, 0, sizeof(gcSockAddr));
	gcSockAddr.sin_family = AF_INET;
	gcSockAddr.sin_addr.s_addr = inet_addr(target_ip);
	gcSockAddr.sin_port = htons(toPort);
	
	//for (;;) 
  while(listenWorkerThreadShouldRun)
  {
		
		/*Send Heartbeat */
		mavlink_msg_heartbeat_pack(mavlinkSystemId, mavlinkComponentId, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcSockAddr, sizeof(struct sockaddr_in));
		
		/* Send Status */
		mavlink_msg_sys_status_pack(mavlinkSystemId, mavlinkComponentId, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcSockAddr, sizeof (struct sockaddr_in));
		
		/* Send Local Position */
		mavlink_msg_local_position_ned_pack(mavlinkSystemId, mavlinkComponentId, &msg, microsSinceEpoch(), 
										position[0], position[1], position[2],
										position[3], position[4], position[5]);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcSockAddr, sizeof(struct sockaddr_in));
		
		/* Send attitude */
		mavlink_msg_attitude_pack(mavlinkSystemId, mavlinkComponentId, &msg, microsSinceEpoch(), 1.2, 1.7, 3.14, 0.01, 0.02, 0.03);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(sock, buf, len, 0, (struct sockaddr*)&gcSockAddr, sizeof(struct sockaddr_in));

		//* @param time_boot_ms [ms] Timestamp (time since system boot).
 		//* @param lat [degE7] Latitude, expressed
 		//* @param lon [degE7] Longitude, expressed
 		//* @param alt [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
 		//* @param relative_alt [mm] Altitude above ground
 		//* @param vx [cm/s] Ground X Speed (Latitude, positive north)
 		//* @param vy [cm/s] Ground Y Speed (Longitude, positive east)
 		//* @param vz [cm/s] Ground Z Speed (Altitude, positive down)
 		//* @param hdg [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX

		int32_t lat = 47.4684732 *10000000L;
		int32_t lon = -121.7676069 *10000000L;
		int32_t alt = 50.0000000;
		int32_t relative_alt = 0.0;
		int32_t vx = 0;
		int16_t vy = 0;
		int16_t vz = 0;
		int16_t hdg = 10 * 100;

		mavlink_msg_global_position_int_pack(mavlinkSystemId, mavlinkComponentId, &msg, 
			microsSinceEpoch(), lat, lon, alt, relative_alt, vx, vy, vz, hdg );

		sendMavMessageToGcs(&msg);
					
		memset(buf, 0, BUFFER_LENGTH);
		sleep(1); // Sleep one second
  }
}

}