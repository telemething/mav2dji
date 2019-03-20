/*
 * mav2dji_ros.cpp
 *
 *  Created on: 1.1.1
 *      Author: Mark West
 *   
 */

#include <mav_udp.hpp>
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

// found this but I dont think it's current https://subak.io/code/px4/Firmware/build_px4fmu-v2_default/build_git_version.h.html
//#define FIRMWARE_BUILD_VERSION 0x9c2dd48814a6ade1

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

mav_udp::mav_udp()
{
  init();
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

/*mav_udp::mav_udp(mavMessageCallbackType callback)
{
	addMavMessageCallback(callback);
	init();
}*/

//*****************************************************************************
//*
//*
//*
//******************************************************************************

mav_udp::~mav_udp()
{
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void mav_udp::init()
{
		verbose = false;

    vehicleUdpAddress = "";
    vehicleUdpPort = 14551;
    qgcUdpAddress = "127.0.0.1";
    qgcUdpPort = 14550;

		mavlinkSystemId = 1;
		mavlinkComponentId = 1;

		/*union 
		{
			unsigned long gvbul = FIRMWARE_BUILD_VERSION;
			uint8_t gvbui8[8];
		} xvert;

		px4_git_version_binary = xvert.gvbui8;*/
}

//uint8_t* mav_udp::getGitVersion()
//{
//	return px4_git_version_binary;
//}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void mav_udp::addGotMavMessageCallback(MavlinkMessageInfo::mavMessageCallbackType callback)
{
	mavMessageCallback = callback;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

uint64_t mav_udp::microsSinceEpoch()
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

MavUdpRet mav_udp::startConnection()
{
    listenWorkerThreadShouldRun = true;

		vehicleMavlinkSocket = createSocket(vehicleUdpAddress, vehicleUdpPort, true);

    listenWorkerThread = std::thread(
			&mav_udp::listenWorker, this, vehicleMavlinkSocket, qgcUdpAddress, qgcUdpPort);
    sendWorkerThread = std::thread(
			&mav_udp::exampleLoop, this, vehicleMavlinkSocket, qgcUdpAddress, qgcUdpPort);

		return MavUdpRet();
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

void mav_udp::stopConnection()
{
    listenWorkerThreadShouldRun = false;
}

//*****************************************************************************
//*
//*
//*
//******************************************************************************

int mav_udp::sendMavMessageToGcs(const mavlink_message_t* pMsg)
{
	// one thread at a time, lck unlocks when out of scope
	std::unique_lock<std::mutex> lck (sendMavMessageToGcsMutex);

	auto len = mavlink_msg_to_send_buffer(udpSendBuffer, pMsg);

	auto bytes_sent = sendto(vehicleMavlinkSocket, udpSendBuffer, len, 0, 
		(struct sockaddr*)&gcSockAddr, sizeof(struct sockaddr_in));

	// we reuse the same buffer every time, clean it
	memset(udpSendBuffer, 0, BUFFER_LENGTH);
}

//*****************************************************************************
//*
//*	Params: address
//*					port : should be 14551
//*
//******************************************************************************

int mav_udp::createSocket(std::string localAddress, int localPort, bool blocking)
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

void mav_udp::listenWorker(int sock, std::string fromAddress, int fromPort)
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

			parseMavlink(MAVLINK_COMM_0, buf, &msg, &status);
				printf("\n---> Port: %i, Magic: %02X, SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", 
					gcPort, msg.magic, msg.sysid, msg.compid, msg.len, msg.msgid);

			for (i = 0; i < recsize; ++i)
			{
				temp = buf[i];

				if(verbose)
					printf("%02x ", (unsigned char)temp);

				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
				{
					// Packet received
					if(verbose)
						printf("\n-->: Port: %i, Magic: %02X, SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", 
							gcPort, msg.magic, msg.sysid, msg.compid, msg.len, msg.msgid);

					mavMessageCallback(&msg);
				}
				else
				{
					//printf("----- unparsable packet received ------\n");
					//parseMavlink(MAVLINK_COMM_1, buf[i], &msg, &status);
					//printf("\nReceived packet: Port: %i, SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", 
					//		gcPort, msg.sysid, msg.compid, msg.len, msg.msgid);
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
//* mavlink.io protocol overview : https://mavlink.io/en/about/overview.html
//* mavlink devguide packet serialization : https://github.com/mavlink/mavlink-devguide/blob/master/en/guide/serialization.md
//* message list : https://mavlink.io/en/messages/common.html
//*
//******************************************************************************

void mav_udp::parseMavlink(uint8_t chan, uint8_t* in, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
	r_message->magic = *in++; // Mavlink V1 = 0xFE, Mavlink V2 = 0xFD
	r_message->len = *in++;
	r_message->incompat_flags = *in++;
	r_message->compat_flags = *in++;
	r_message->seq = *in++;
	r_message->sysid = *in++;
	r_message->compid = *in++;
	r_message->msgid = *in++;
	r_message->msgid += *in++ << 8;
	r_message->msgid += *in++ << 8;
}

//*****************************************************************************
//*
//* https://github.com/mavlink/mavlink-devguide/blob/master/en/guide/serialization.md
//*
//******************************************************************************

void mav_udp::exampleLoop(int sock, std::string toAddress, int toPort)
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

/*
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
*/
					
		memset(buf, 0, BUFFER_LENGTH);
		sleep(1); // Sleep one second
  }
}

}