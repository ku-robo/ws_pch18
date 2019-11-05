#include <iostream>
#include <boost/asio.hpp>

#include "ros/ros.h"

namespace Okatech
{
class Core
{
public:
    std::string frame_id = "odom";
    std::string child_frame_id = "base_link";
};

class Serial
{
    boost::asio::io_service iosev;
    boost::asio::serial_port sp = boost::asio::serial_port(iosev);

public:
    std::string port = "/dev/ttyUSB0";
    int rate = 19200;
    bool open()
    {
        using namespace boost::asio;
        boost::system::error_code ret;
        sp.set_option(serial_port::baud_rate(rate)); //波特率
        sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
        sp.set_option(serial_port::parity(serial_port::parity::none));
        sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
        sp.set_option(serial_port::character_size(8));
        sp.open(port, ret);
        if (ret)
        {
            std::cerr << "打开okatech串口失败: " << ret << std::endl;
            return false;
        }
    }
    void write(std::string msg)
    {
        boost::asio::async_write(sp, boost::asio::buffer(msg), msg.size());
    }
};

} // namespace Okatech

int main(int argc, char **argv)
{
    Okatech::Core core;
    Okatech::Serial serial;

    ros::init(argc, argv, "okatech");
    ros::NodeHandle node;
    node.param("port", serial.port, serial.port);
    node.param("frame_id", core.frame_id, core.frame_id);
    node.param("child_frame_id", core.child_frame_id, core.child_frame_id);

    if (!serial.open())
        exit(0);

    ros::spin();
    return 0;
}
