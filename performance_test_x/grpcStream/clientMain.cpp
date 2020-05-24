#include <chrono>
#include <grpcpp/grpcpp.h>
#include "Message.grpc.pb.h"
int main(int argc, char** argv)
{
    std::shared_ptr<grpc::Channel> channel = grpc::CreateChannel("0.0.0.0:30002", grpc::InsecureChannelCredentials());
    std::unique_ptr<MyServer::Stub> stub(MyServer::NewStub(channel));

    StreamRequest request;
    grpc::ClientContext context;
    
    TopMessage streamedReply;
    int messageCount = 0;

    // Start requesting streaming.
    auto start = std::chrono::high_resolution_clock::now();
    std::unique_ptr<grpc::ClientReader<TopMessage>> reader(stub->sendStreamRequest(&context, request));
    while (reader->Read(&streamedReply)) 
    {
        // No need to de-serialize the message. Already de-serialized.
        ++messageCount;

        //std::cout << "Received streamed message: ===================== " << std::endl;
        //std::cout << streamedReply.DebugString() << std::endl;
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto durationUs = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    int numMessagesExchanged = messageCount;
    std::cout << "Stats: " << durationUs.count() << " us for " << numMessagesExchanged << " messages each with " << streamedReply.ByteSizeLong() << " bytes" << std::endl;
    std::cout << "Latency: " << (double)(durationUs.count()) / (double)(numMessagesExchanged) << " usec" << std::endl;
    std::cout << "Throughput: " << (double)(numMessagesExchanged) / (double)(durationUs.count() / 1e6) << " messages/sec" << std::endl;
    // Stats: 2955491 us for 200000 messages each with 71 bytes
    // Latency: 14.7775 usec
    // Throughput: 67670.7 messages/sec

    return 0;
}
