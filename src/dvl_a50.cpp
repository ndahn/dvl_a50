#include "dvl_a50/dvl_a50.hpp"


using namespace dvl_a50;
using nlohmann::json;


int DvlA50::connect(std::string addr, bool enable)
{
    // Open TCP/IP connection to DVL
    tcp_socket = new TCPSocket((char*)addr.c_str(), 16171);
    if(tcp_socket->Create() < 0) 
    {
        // Error creating the socket
        return -1;
    }

    tcp_socket->SetRcvTimeout(400);
    std::string error;
    
    int error_code = 0;
    //int fault = 1; 
    
    std::chrono::steady_clock::time_point first_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point first_time_error = first_time;

    while(fault != 0)
    {
        fault = tcp_socket->Connect(5000, error, error_code);
        if(error_code == 114)
        {
            // Is the sensor on?
            usleep(2000000);
            std::chrono::steady_clock::time_point current_time_error = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(current_time_error - first_time_error).count();
            if(dt >= 78.5) //Max time to set up
            {
                fault = -10;
                break;
            }
        }
        else if(error_code == 103)
        {
            // No route to host, DVL might be booting?
            usleep(2000000);
        }
    }  
    
    if(fault == -10)
    {
        tcp_socket->Close();
        // Turn the sensor on and try again!
        return -10;
    }
    
    this->enabled = enable;
    if (!enable)
    {
        // Disable transducer operation to limit sensor heating out of water.
        this->set("acoustic_enabled", false);
    }
    usleep(2000);

    return 0;
}

void DvlA50::disconnect()
{
    if (tcp_socket != nullptr)
    {
        tcp_socket->Close();
        delete tcp_socket;
    }
}

void DvlA50::send(const Message& msg)
{
    std::string str = msg.dump();
    char* c = &*str.begin();
    std::lock_guard<std::mutex> lock(mtx);
    tcp_socket->Send(c);
}

DvlA50::Message DvlA50::receive()
{
    // Single threaded access only
    std::lock_guard<std::mutex> lock(mtx);
    
    if(fault != 0) {
        return {"fault", std::to_string(fault)};
    }
    
    std::string str; 
    char c = 0;
    
    while (c != '\n') {
        // Receive 1 byte and check
        int n = tcp_socket-> Receive(&c);
        if (n > 0) {
            str += c;
        }
        else {
            // 0: socket closed, -1: timeout
            break;
        }
    }

    if (str.size() <= 1) {
        return {"error", "no data received"};
    }
    
    try {
        return json::parse(str);
    }
    catch(const std::exception& e) {
        return {"error", std::string(e.what())};
    }
}

DvlA50::Message DvlA50::wait_for_response(std::function<bool(const Message&)> check, uint32_t timeout_ms)
{
    using namespace std::chrono;

    auto start = steady_clock::now();
    auto end = start + std::chrono::milliseconds(timeout_ms);
    Message res;

    do
    {
        if (steady_clock::now() > end)
        {
            throw std::runtime_error("Expected response did not appear in time");
        }

        res = receive();
    }
    while (!check(res));
    
    return res;
}


void DvlA50::configure(
    int speed_of_sound,
    bool acoustic_enabled,
    bool led_enabled,
    int mounting_rotation_offset,
    std::string range_mode)
{
    DvlA50::Message message;
    message["command"] = "set_config";
    message["parameters"]["speed_of_sound"] = speed_of_sound;
    message["parameters"]["acoustic_enabled"] = acoustic_enabled;
    message["parameters"]["dark_mode_enabled"] = led_enabled;
    message["parameters"]["mounting_rotation_offset"] = mounting_rotation_offset;
    message["parameters"]["range_mode"] = range_mode;
    //message["parameters"]["periodic_cycling_enabled"] = true;

    send(message);
}

void DvlA50::set_speed_of_sound(int speed_of_sound)
{
    set("speed_of_sound", speed_of_sound);
}

void DvlA50::set_acoustic_enabled(bool enabled)
{
    set("acoustic_enabled", enabled);
}

void DvlA50::set_led_enabled(bool led_enabled)
{
    set("led_enabled", led_enabled);
}

void DvlA50::set_mounting_rotation_offset(int offset_degrees)
{
    set("mounting_rotation_offset", offset_degrees);
}

void DvlA50::set_range_mode(std::string range_mode)
{
    set("range_mode", range_mode);
}


void DvlA50::send_command(std::string cmd)
{
    std::cout << "send command '" << cmd << "'" << std::endl;

    json json_data;
    json_data["command"] = cmd;
    send(json_data);
}

void DvlA50::get_config()
{
    send_command("get_config");
}

void DvlA50::calibrate_gyro()
{
    send_command("calibrate_gyro");
}

void DvlA50::reset_dead_reckoning()
{
    send_command("reset_dead_reckoning");
}

void DvlA50::trigger_ping()
{
    if (enabled)
    {
        set_acoustic_enabled(false);
    }
    send_command("trigger_ping");
}
