#include <string>
#include <grpcpp/grpcpp.h>
#include "Message.grpc.pb.h"

using grpc::ServerContext;

class ServiceImpl final : public MyServer::Service
{
    grpc::Status sendTopMessage(ServerContext* context, 
                        const TopMessage* request,
                        TopMessage* reply) override 
  {
      // No need to deserialize the incoming request.
      // No need to serialize the outgoing reply, just copy it.
      reply->CopyFrom(*request);
      return grpc::Status::OK;
  }
};

void runServer() 
{
  ServiceImpl service;

  grpc::ServerBuilder builder;

  std::string server_address("127.0.0.1:30002");
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
