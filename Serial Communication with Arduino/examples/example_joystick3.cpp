/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"
#include "serial/serial.h"
#include <math.h>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <string.h>
#include "thread"


using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level, serial::Serial& my_serial): safe(LeggedType::A1), udp(level), my_serial(my_serial){
        udp.InitCmdData(cmd);
    }
    void UDPSend();
    void UDPRecv();
    void RobotControl();
    void SendToRobot(std::string data);

    serial::Serial& my_serial;
    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    xRockerBtnDataStruct _keyData;
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};

void Custom::UDPRecv()
{ 
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    motiontime++;
    udp.GetRecv(state);

    memcpy(&_keyData, state.wirelessRemote, 40);

    //  std::cout << "lx: " << _keyData.lx << std::endl;
    //std::cout << "ly: " << _keyData.ly << std::endl;
	//std::cout << "rx: " << _keyData.rx << std::endl;
	//std::cout << "ry: " << _keyData.ry << std::endl;
	//std::cout << "  " << std::endl;
    //SendToRobot(std::to_string(_keyData.lx));

	std::cout <<_keyData.lx<< "," << _keyData.ly <<"," <<_keyData.rx<<"," << _keyData.ry <<std::endl;


    std::string Lx = std::to_string(_keyData.lx);
    Lx = Lx.substr(0, Lx.find(".") + 3);
    std::string Ly = std::to_string(_keyData.ly);
    Ly = Ly.substr(0, Ly.find(".") + 3);
    std::string Rx = std::to_string(_keyData.rx);
    Rx = Rx.substr(0, Rx.find(".") + 3);
    std::string Ry = std::to_string(_keyData.ry);
    Ry = Ry.substr(0, Ry.find(".") + 3);
    std::cout <<Lx<< "," << Ly <<"," <<Rx<<"," << Ry <<std::endl;

    // double llx = _keyData.lx;
    // std::ostringstream stream;
    // stream << std::fixed << std::setprecision(2) << llx;
    // std::string Lx = stream.str();
    // double lly = _keyData.ly;
    // stream << std::fixed << std::setprecision(2) << lly;
    // std::string Ly = stream.str();
    // double rrx = _keyData.rx;
    // stream << std::fixed << std::setprecision(2) << rrx;
    // std::string Rx = stream.str();
    // double rry = _keyData.ry;
    // stream<< std::fixed << std::setprecision(2) << rry;
    // std::string Ry = stream.str();



    std::string data = Lx+", "+Ly+", "+Rx+", "+Ry+"\n";
    SendToRobot(data);
 
    //udp.SetSend(cmd);
}

void Custom::SendToRobot(std::string stringData)
{
    if (my_serial.isOpen())
    {
        std::cout << "Port opened succesfully" << std::endl;
    }
    else
    {
        std::cout << "Port failed to open" << std::endl;
    }
    my_serial.flushOutput();


        size_t bytesWritten = my_serial.write(stringData);

        // std::string result = my_serial.read(test_string.length() + 1);
        std::cout << "Bytes sent: " << bytesWritten << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
   
}

int main(void)
{
    std::cout << "Communication level is set to  HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

///////////////////////////////////////////////////////////////////////////////////////////
    //std::string str = std::to_string(_keyData.lx);

   serial::Serial my_serial("/dev/ttyUSB0", 19200, serial::Timeout::simpleTimeout(3000));
    
///////////////////////////////////////////////////////////////////////////////////////////////
    Custom custom(HIGHLEVEL, my_serial);
    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
