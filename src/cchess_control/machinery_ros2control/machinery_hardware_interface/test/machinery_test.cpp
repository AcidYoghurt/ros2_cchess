#include <limits>
#include <vector>
#include <memory>
#include <functional>
#include <libserial/SerialPort.h>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <regex>
#include <thread>
#include <unistd.h>

using namespace std;
LibSerial::SerialPort serial_port;  // 串口
vector<double> hw_commands_;

void Send() {
    hw_commands_ = {10.0,10.0,10.0};

    std::string command = "JointAngleOffset_"+std::to_string(hw_commands_[0])+","+
                                          std::to_string(hw_commands_[1])+","+
                                          std::to_string(hw_commands_[2])+","+"100";
    while (1) {
        serial_port.Write(command);
        serial_port.DrainWriteBuffer();
        sleep(1);
    }
}


void Reveive() {
    while (1) {
        std::string response;
        try {
            serial_port.ReadLine(response,'\r',100);
            cout<<response<<endl;
        }catch (const exception& e) {
        }
    }
}


int main() {
    serial_port.Open("/dev/ttyUSB0");
    serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    serial_port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_port.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

    thread thread1(Reveive);

}

