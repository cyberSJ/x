#include "zmq.hpp"
#include <iostream>
#include <unistd.h>
#include <string>
#include <thread>
#include <chrono>
#ifndef _WIN32
	#include <unistd.h>
#else
	#include <windows.h>
#endif

// Server using ZMQ

int main(int argc, char** argv)
{
	// Prepare our zmq context and socket
	zmq::context_t context(1);
	zmq::socket_t socket(context, ZMQ_REP);
	socket.bind("tcp://*:5555");

	while (true){
		zmq::message_t request;

		socket.recv(&request);
		std::cout << "Received Hello" << std::endl;
	
		// Do some work
		#ifdef _WIN32
			sleep(1);
		#else
			std::this_thread::sleep_for(std::chrono::seconds(1));	
		#endif

		zmq::message_t reply(5);
		memcpy((void *) reply.data(), "World", 5);
		socket.send(reply);
	}

	return 0;
}
