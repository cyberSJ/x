#include <string>
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

class ServiceImpl final : public MyServer::Service
{
  public:
    ServiceImpl()
      : messageToStream(prepareMessage())
    {
    }

    grpc::Status sendTopMessage(grpc::ServerContext* context, 
                                const TopMessage* request,
                                TopMessage* reply) override 
    {
        // No-op for this example.
        return grpc::Status::OK;
    }

    grpc::Status sendStreamRequest(grpc::ServerContext* context,
                                   const StreamRequest* request,
                                   grpc::ServerWriter<TopMessage>* writer) override
    {
        int repetition = 2e5;
        for (int i = 0; i < repetition; ++i)
        {
            writer->Write(messageToStream);
        }

        return grpc::Status::OK;
    }

  private:
    TopMessage messageToStream;
};

void runServer() 
{
  ServiceImpl service;

  grpc::ServerBuilder builder;

  std::string server_address("0.0.0.0:30002");
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);

  std::unique_ptr<grpc::Server> server(builder.BuildAndStart());

  std::cout << "Server listening on " << server_address << std::endl;
  server->Wait();
}


int main(int argc, char** argv)
{
    runServer();
    return 0;
}
