
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include "../message/Message.pb.h"

using boost::asio::ip::tcp;

enum { max_length = 1024 };

class session
{
public:
  session(boost::asio::io_service& io_service)
    : socket_(io_service),
      top(prepareMessage()),
      messageSizeBytes(top.ByteSize())
  {
  }

  tcp::socket& socket()
  {
    return socket_;
  }

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

  void start()
  {
    while (true)
    {
        // Keep giving back a response to the client.
        boost::asio::read(socket_, boost::asio::buffer(reply, messageSizeBytes));
        top.ParseFromString(reply);

        top.SerializeToString(&message);
        boost::asio::write(
            socket_,
            boost::asio::buffer(message));
    }

    //auto stop = std::chrono::high_resolution_clock::now();
    //auto durationUs = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    //auto now = std::chrono::system_clock::now().time_since_epoch();
    //top.set_timestamp(now.count());

    //top.SerializeToString(&message);
    //boost::asio::write(
    //    socket_,
    //    boost::asio::buffer(message));
    //std::cout << "Finished sending." << std::endl;
    //std::cout << "Throughput: " << durationUs.count() << " us for " << repetition << " messages each with 71 bytes" << std::endl;
    // Throughput: 2412389 us for 1000000 messages each with 71 bytes

    //socket_.async_read_some(boost::asio::buffer(data_, max_length),
    //    boost::bind(&session::handle_read, this,
    //      boost::asio::placeholders::error,
    //      boost::asio::placeholders::bytes_transferred));
  }

  //void exchangeData()
  //{
  //  top.SerializeToString(&message);
  //  boost::asio::write(
  //      socket_,
  //      boost::asio::buffer(message));

  //  boost::asio::read(socket_, boost::asio::buffer(reply.data(), messageSizeBytes));
  //  top.ParseFromString(reply);
  //  std::cout << "Server received client response: " << top.DebugString() << std::endl;
  //}

private:
  void handle_read(const boost::system::error_code& error,
      size_t bytes_transferred)
  {
    if (!error)
    {
      boost::asio::async_write(socket_,
          boost::asio::buffer(data_, bytes_transferred),
          boost::bind(&session::handle_write, this,
            boost::asio::placeholders::error));
    }
    else
    {
      delete this;
    }
  }

  void handle_write(const boost::system::error_code& error)
  {
	delete this;
	return;

    if (!error)
    {
      socket_.async_read_some(boost::asio::buffer(data_, max_length),
          boost::bind(&session::handle_read, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
    }
    else
    {
      delete this;
    }
  }

  tcp::socket socket_;
  enum { max_length = 1024 };
  char data_[max_length];
  TopMessage top;
  int messageSizeBytes;
  std::string message;
  char reply[max_length];
};

class server
{
public:
  server(boost::asio::io_service& io_service, short port)
    : io_service_(io_service),
      acceptor_(io_service, tcp::endpoint(tcp::v4(), port))
  {
    start_accept();
  }

private:
  void start_accept()
  {
    session* new_session = new session(io_service_);
    acceptor_.async_accept(new_session->socket(),
        boost::bind(&server::handle_accept, this, new_session,
          boost::asio::placeholders::error));
  }

  void handle_accept(session* new_session,
      const boost::system::error_code& error)
  {
    if (!error)
    {
      new_session->start();
    }
    else
    {
      delete new_session;
    }

    start_accept();
  }

  boost::asio::io_service& io_service_;
  tcp::acceptor acceptor_;
};

int main(int argc, char* argv[])
{
  try
  {
    boost::asio::io_service io_service;

    using namespace std; // For atoi.
    server s(io_service, 30000);

    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}
