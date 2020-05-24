#include <chrono>
#include <iostream>
#include <cpprest/http_client.h>
#include "../message/Message.pb.h"

using namespace utility;                    // Common utilities like string conversions
using namespace web;                        // Common features like URIs.
using namespace web::http;                  // Common HTTP functionality
using namespace web::http::client;          // HTTP client features
using namespace concurrency::streams;       // Asynchronous streams

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

class Client
{
  public:
    Client()
        : client(U("http://127.0.0.1:30001/")),
          builder(U("/")),
          top(prepareMessage()),
          size(top.ByteSize()),
          buffer(size)
    {
    }

    void request()
    {
        top.SerializeToArray(buffer.data(), size);
        auto instream = Concurrency::streams::bytestream::open_istream(buffer);

        client.request(methods::PUT, builder.to_string(), instream)
              .then([=](http_response response)
               {
                    const std::string body = response.content_ready().get().extract_utf8string(true).get();

                    TopMessage top;
                    top.ParseFromString(body);
                    //std::cout << "Received from server----- " << top.DebugString() << std::endl;
               })
              .wait();
    }

    size_t getMessageSizeBytes() const
    {
        return size;
    }

  private:
    http_client client;
    uri_builder builder;
    TopMessage top;
    size_t size;
    std::vector<unsigned char> buffer;
};

int main(int argc, char** argv)
{
    Client client;

    int repetition = 1e2;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repetition; ++i)
    {
        client.request();
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto durationUs = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    int numMessagesExchanged = 2 * repetition;
    std::cout << "Stats: " << durationUs.count() << " us for " << numMessagesExchanged << " messages each with " << client.getMessageSizeBytes() << " bytes" << std::endl;
    std::cout << "Latency: " << (double)(durationUs.count()) / (double)(numMessagesExchanged) << " usec" << std::endl;
    std::cout << "Throughput: " << (double)(numMessagesExchanged) / (double)(durationUs.count() / 1e6) << " messages/sec" << std::endl;
    // Stats: 8725855 us for 200 messages each with 71 bytes
    // Latency: 43629.3 usec
    // Throughput: 22.9204 messages/sec

    return 0;
}
