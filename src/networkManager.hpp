#ifndef SRC_NETWORKMANAGER_HPP_
#define SRC_NETWORKMANAGER_HPP_

#include <WPILib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>

namespace FRC
{
	class networkManager
	{
	public:
		networkManager();

		sockaddr_in server;
		unsigned short port = 5800;
		int udpSock = -1;
		socklen_t serverLen = 0;

		int xPos = -1;
		int yPos = -1;
		int dist = -1;


		int netSetup();
		int grabData();
		int closeNet();
		void sendData(char * data);

	private:


	};
}



#endif /* SRC_NETWORKMANAGER_HPP_ */
