#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include "../message/Message.pb.h"

using boost::asio::ip::tcp;

enum { max_length = 1024 };

TopMessage prepareMessage()
{
  TopMessage top;
  top.set_kind(Kind::KIND_TWO);

  top.set_type(SubMessageType::CAT_TWO);

  top.set_timestamp(std::chrono::system_clock::now().time_since_epoch().count());

  SubMessageOne* smo = top.mutable_sub_message_one();
  Timestamp* timestamp = smo->mutable_timestamp();
  timestamp->set_seconds(123);
  timestamp->set_nanoseconds(456);
  smo->set_f(1.234);
  smo->set_b(1231.123);
  smo->set_s(3.4);
  smo->set_x("this is x");
  smo->set_y("this is y");
  smo->set_z("this is z");
  std::cout << top.DebugString() << std::endl;
  std::cout << "Byte size: " << top.ByteSize() << std::endl;

  return top;
}

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

    int repetition = 1e5;
    int messageSizeBytes = 71;
    std::string receivedMessage;
    TopMessage top = prepareMessage();
    std::string message;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repetition; ++i)
    {
        top.SerializeToString(&message);
        boost::asio::write(
            s,
            boost::asio::buffer(message));

        boost::asio::read(s, boost::asio::buffer(reply, messageSizeBytes));
        top.ParseFromString(reply);


        //std::cout << "Reply is: " << top.DebugString() << std::endl;
        //std::cout.write(reply, reply_length);
        //std::cout << "\n";
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto durationUs = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    int numMessagesExchanged = 2 * repetition;
    std::cout << "Stats: " << durationUs.count() << " us for " << numMessagesExchanged << " messages each with " << messageSizeBytes << " bytes" << std::endl;
    std::cout << "Latency: " << (double)(durationUs.count()) / (double)(numMessagesExchanged) << " usec" << std::endl;
    std::cout << "Throughput: " << (double)(numMessagesExchanged) / (double)(durationUs.count() / 1e6) << " messages/sec" << std::endl;
    // Stats: 2762441 us for 200000 messages each with 71 bytes
    // Latency: 13.8122 usec
    // Throughput: 72399.7 messages/sec

    //boost::asio::read(s, boost::asio::buffer(reply, messageSizeBytes));
    //top.ParseFromString(reply);
    //auto receivedTime = std::chrono::system_clock::now().time_since_epoch();
    //auto latencyNs = receivedTime.count() - top.timestamp();

    //auto durationUs = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    //std::cout << "Throughput: " << durationUs.count() << " us for " << repetition << " messages each with " << messageSizeBytes << " bytes" << std::endl;
    // Throughput: 2492734 us for 1000000 messages each with 71 bytes
    
    //std::cout << "Latency: " << latencyNs << " ns" << std::endl;
    // Latency: 80195222 ns
    // But this is probably the deserialization time + other things, rather than network packet travel time.

	//std::cout << "Press ENTER to exit." << std::endl;
    //std::string line;
    //std::getline(std::cin, line);
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}
