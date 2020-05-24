#include <chrono>
#include <grpcpp/grpcpp.h>
#include "Message.grpc.pb.h"

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
  std::cout << "Byte size: " << top.ByteSizeLong() << std::endl;

  return top;
}

int main(int argc, char** argv) 
{
    std::shared_ptr<grpc::Channel> channel = grpc::CreateChannel("0.0.0.0:30002", grpc::InsecureChannelCredentials());
    std::unique_ptr<MyServer::Stub> stub(MyServer::NewStub(channel));

    TopMessage request = prepareMessage();
    TopMessage reply;

    int repetition = 1e4;
    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < repetition; ++i)
    {
        grpc::ClientContext context;
        stub->sendTopMessage(&context, request, &reply);
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto durationUs = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    int numMessagesExchanged = 2 * repetition;
    std::cout << "Stats: " << durationUs.count() << " us for " << numMessagesExchanged << " messages each with " << request.ByteSizeLong() << " bytes" << std::endl;
    std::cout << "Latency: " << (double)(durationUs.count()) / (double)(numMessagesExchanged) << " usec" << std::endl;
    std::cout << "Throughput: " << (double)(numMessagesExchanged) / (double)(durationUs.count() / 1e6) << " messages/sec" << std::endl;
    // Stats: 2200246 us for 20000 messages each with 71 bytes
    // Latency: 110.012 usec
    // Throughput: 9089.89 messages/sec
}
