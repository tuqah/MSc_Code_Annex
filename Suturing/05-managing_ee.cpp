#include <DeviceManagerClientRpc.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <thread>
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>

#include "utilities.h"

#include <boost/asio.hpp>
#include <stack>
#include <ctime>

using namespace boost::asio;

/// this code was used before the suturing experiment to test
/// the responsivity of the arduino board and ready the end-effector for the action
int main(){
    // connect to the arduino board of the end-effector
    io_service io;
    serial_port port(io, "COM3"); //connect to the correct port where the end-effector is attached
    port.set_option(serial_port_base::baud_rate(9600)); // specify the correct baud rate
    string tool = "2\n"; // in this experiment, the needle holder (2) was used, so this was defined
    string open = "o\n"; // to open the end-effector
    string close = "c\n"; // to close the end-effector
    // call to the arduino board and specify the tool that will be used
    write(port, buffer(tool.c_str(), tool.size()));
    // open the end-effector
    write(port, buffer(open.c_str(), open.size()));

    std::cout<<"Type Y to Close."<<std::endl;
    char answer;
    char response = 'Y';
    std::cin>>answer;
    if(answer==response) {
        // close the end-effector after setting up the instrument and the needle
        write(port, buffer(close.c_str(), close.size()));
    }
    return 0;
}