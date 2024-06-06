#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <iostream>
#include <boost/asio.hpp>
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <termios.h>
#include <unistd.h>

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
  NtripClient(const std::string& username, const std::string& password) 
  : Node("ntrip_client"), username_(username), password_(password), rtcm_count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("rtcm_data", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&NtripClient::timer_callback, this));
  }

private:
  void connect_to_ntrip() {
    try {
      const std::string host = "212.156.70.42"; // IP adresi buraya
      const std::string port = "2101"; // Port buraya
      const std::string mountpoint = "VRSRTCM34"; // Mount point buraya

      boost::asio::io_context io_context;
      tcp::resolver resolver(io_context);
      tcp::resolver::results_type endpoints = resolver.resolve(host, port);

      socket_ = std::make_unique<tcp::socket>(io_context);
      boost::asio::connect(*socket_, endpoints);

      std::string credentials = username_ + ":" + password_;
      std::string encoded_credentials = encode_base64(credentials);

      std::string request = "GET /" + mountpoint + " HTTP/1.1\r\n"
                            "Host: " + host + "\r\n"
                            "Ntrip-Version: Ntrip/2.0\r\n"
                            "User-Agent: NTRIP client\r\n"
                            "Authorization: Basic " + encoded_credentials + "\r\n"
                            "Connection: keep-alive\r\n"
                            "\r\n";
      boost::asio::write(*socket_, boost::asio::buffer(request));
      RCLCPP_INFO(this->get_logger(), "NTRIP sunucusuna bağlantı isteği gönderildi.");

      char response[1024];
      size_t len = socket_->read_some(boost::asio::buffer(response));
      std::cout.write(response, len);
      RCLCPP_INFO(this->get_logger(), "NTRIP sunucusundan cevap alındı: %s", std::string(response, len).c_str());

      // Sabit NMEA GGA cümlesi
      nmea_gga_ = "$GPGGA,143812,4105.314,N,02839.306,E,2,4,1.5,0,M,0,M,,*4F\r\n";
      boost::asio::write(*socket_, boost::asio::buffer(nmea_gga_));
      RCLCPP_INFO(this->get_logger(), "NMEA GGA cümlesi gönderildi: %s", nmea_gga_.c_str());

    } catch (std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Bağlantı hatası: %s", e.what());
      reconnect_timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&NtripClient::connect_to_ntrip, this));
    }
  }

  void timer_callback() {
    if (!socket_ || !socket_->is_open()) {
      connect_to_ntrip();
      return;
    }

    try {
      // RTCM verilerini oku
      char buf[2048];
      size_t len = socket_->read_some(boost::asio::buffer(buf));
      if (len > 0) {
        std_msgs::msg::ByteMultiArray msg;
        msg.data.insert(msg.data.end(), buf, buf + len);
        publisher_->publish(msg);

        // RTCM mesajlarını terminale yazdırma
        RCLCPP_INFO(this->get_logger(), "RTCM yakalandı. Mesaj sayısı: %d", ++rtcm_count_);
      } else {
        RCLCPP_WARN(this->get_logger(), "Boş veri alındı, yeniden bağlanılıyor...");
        socket_->close();
      }
    } catch (std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Veri okuma hatası: %s", e.what());
      socket_->close();
      reconnect_timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&NtripClient::connect_to_ntrip, this));
    }
  }

  std::string username_;
  std::string password_;
  int rtcm_count_;
  std::unique_ptr<tcp::socket> socket_;
  std::string nmea_gga_;
  rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::string username, password;

  std::cout << "Enter username: ";
  std::getline(std::cin, username);

  std::cout << "Enter password: ";
  std::getline(std::cin, password);

  auto node = std::make_shared<NtripClient>(username, password);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
