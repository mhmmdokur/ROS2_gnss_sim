#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <iostream>
#include <boost/asio.hpp>
#include <string>
#include <sstream>
#include <iomanip>

using boost::asio::ip::tcp;

static const std::string base64_chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

std::string encode_base64(const std::string &in) {
    std::string out;
    int val = 0, valb = -6;
    for (unsigned char c : in) {
        val = (val << 8) + c;
        valb += 8;
        while (valb >= 0) {
            out.push_back(base64_chars[(val >> valb) & 0x3F]);
            valb -= 6;
        }
    }
    if (valb > -6) out.push_back(base64_chars[((val << 8) >> (valb + 8)) & 0x3F]);
    while (out.size() % 4) out.push_back('=');
    return out;
}

class NtripClient : public rclcpp::Node {
public:
  NtripClient(const std::string& host, const std::string& port, const std::string& username, const std::string& password) 
  : Node("ntrip_client"), host_(host), port_(port), username_(username), password_(password) {
    publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("rtcm_data", 10);
    connect_to_ntrip();
  }

private:
  void connect_to_ntrip() {
    try {
      boost::asio::io_context io_context;
      tcp::resolver resolver(io_context);
      tcp::resolver::results_type endpoints = resolver.resolve(host_, port_);

      tcp::socket socket(io_context);
      boost::asio::connect(socket, endpoints);

      std::string credentials = username_ + ":" + password_;
      std::string encoded_credentials = encode_base64(credentials);

      std::string request = "GET /mountpoint HTTP/1.1\r\n"
                            "Host: " + host_ + "\r\n"
                            "Ntrip-Version: Ntrip/2.0\r\n"
                            "User-Agent: NTRIP client\r\n"
                            "Authorization: Basic " + encoded_credentials + "\r\n"
                            "\r\n";
      boost::asio::write(socket, boost::asio::buffer(request));

      char response[1024];
      size_t len = socket.read_some(boost::asio::buffer(response));
      std::cout.write(response, len);

      while (true) {
        char buf[512];
        size_t len = socket.read_some(boost::asio::buffer(buf));
        // std_msgs::msg::String msg;
        // msg.data = std::string(buf, len);
        std_msgs::msg::ByteMultiArray msg;
        msg.data.insert(msg.data.end(), buf, buf + len);
        publisher_->publish(msg);
      }
    } catch (std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }
  }

  std::string host_;
  std::string port_;
  std::string username_;
  std::string password_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::string host, port, username, password;

  std::cout << "Enter IP address: ";
  std::getline(std::cin, host);

  std::cout << "Enter port: ";
  std::getline(std::cin, port);

  std::cout << "Enter username: ";
  std::getline(std::cin, username);

  std::cout << "Enter password: ";
  std::getline(std::cin, password);

  auto node = std::make_shared<NtripClient>(host, port, username, password);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
