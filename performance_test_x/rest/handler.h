#ifndef HANDLER_H
#define HANDLER_H
#include <iostream>
#include "cpprest/http_listener.h"
#include "cpprest/uri.h"
#include "../message/Message.pb.h"

using namespace web;
using namespace http;
using namespace utility;
using namespace http::experimental::listener;

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
  std::cout << "Server. Byte size: " << top.ByteSize() << std::endl;

  return top;
}


class handler
{
  public:
	handler(utility::string_t url)
		: m_listener(url),
          top(prepareMessage()),
          size(top.ByteSize()),
          buffer(size)
	{
		m_listener.support(methods::PUT, 
						   std::bind(&handler::handle_put, this, std::placeholders::_1));
		m_listener.support(methods::GET, 
						   std::bind(&handler::handle_get, this, std::placeholders::_1));
	}

	pplx::task<void> open()
	{
		return m_listener.open();
	}

	pplx::task<void> close()
	{
		return m_listener.close();
	}	

  private:
    void handle_put(http_request message)
	{
		//std::cout << "handle_put() called" << std::endl;
		const std::string body = message.content_ready().get().extract_utf8string(true).get();

		TopMessage top;
		top.ParseFromString(body);
		//std::cout << "Received: " << top.DebugString() << std::endl;
		

        top.SerializeToArray(buffer.data(), size);
        auto instream = Concurrency::streams::bytestream::open_istream(buffer);
		//size_t sizeBytes = 71;
		//Concurrency::streams::container_buffer<std::vector<unsigned char> > inBuffer;
		//message.body().read_to_end(inBuffer).then([](size_t bytesRead){
		//	
		//});
		message.reply(status_codes::OK, instream);
	}

    void handle_get(http_request message)
	{
		std::cout << "handle_get() called" << std::endl;
		message.reply(status_codes::OK, "server replies");
	}

    http_listener m_listener;
    TopMessage top;
    size_t size;
    std::vector<unsigned char> buffer;
};

#endif // HANDLER_H
