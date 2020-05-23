#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include "../message/Message.pb.h"

using boost::asio::ip::tcp;

enum { max_length = 1024 };

int main(int argc, char* argv[])
{
  try
  {
    boost::asio::io_service io_service;

    tcp::resolver resolver(io_service);
    tcp::resolver::query query(tcp::v4(), "localhost", "30000");
    tcp::resolver::iterator iterator = resolver.resolve(query);

    tcp::socket s(io_service);
    boost::asio::connect(s, iterator);

    using namespace std; // For strlen.
    //std::cout << "Enter message: ";
    //char request[max_length];
    //std::cin.getline(request, max_length);
    //size_t request_length = strlen(request);
    //boost::asio::write(s, boost::asio::buffer(request, request_length));

    char reply[max_length];

    int repetition = 1e6;
    int messageSizeBytes = 71;
    std::string receivedMessage;
    TopMessage top;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repetition; ++i)
    {
        boost::asio::read(s, boost::asio::buffer(reply, messageSizeBytes));

        top.ParseFromString(reply);

        //std::cout << "Reply is: " << top.DebugString() << std::endl;
        //std::cout.write(reply, reply_length);
        //std::cout << "\n";
    }
    auto stop = std::chrono::high_resolution_clock::now();

    boost::asio::read(s, boost::asio::buffer(reply, messageSizeBytes));
    top.ParseFromString(reply);
    auto receivedTime = std::chrono::system_clock::now().time_since_epoch();
    auto latencyNs = receivedTime.count() - top.timestamp();

    auto durationUs = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Throughput: " << durationUs.count() << " us for " << repetition << " messages each with " << messageSizeBytes << " bytes" << std::endl;
    // Throughput: 2492734 us for 1000000 messages each with 71 bytes
    
    std::cout << "Latency: " << latencyNs << " ns" << std::endl;
    // Latency: 80195222 ns
    // But this is probably the deserialization time + other things, rather than network packet travel time.
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}
