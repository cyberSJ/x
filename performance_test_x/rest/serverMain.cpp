#include "cpprest/http_listener.h"
#include "cpprest/uri.h"
#include "handler.h"

using namespace web;
using namespace http;
using namespace utility;
using namespace http::experimental::listener;

std::unique_ptr<handler> g_httpHandler;

int main(int argc, char** argv)
{
    utility::string_t address = U("http://127.0.0.1:");
    utility::string_t port = U("30001");
    address.append(port);
	uri_builder uri(address);

	auto addr = uri.to_uri().to_string();
	g_httpHandler = std::unique_ptr<handler>(new handler(addr));
	g_httpHandler->open().wait();

	std::cout << utility::string_t(U("Listening for requests at: ")) << addr << std::endl;

	std::cout << "Press ENTER to exit." << std::endl;
    std::string line;
    std::getline(std::cin, line);

	g_httpHandler->close().wait();
    return 0;
}
