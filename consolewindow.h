#ifndef CONSOLEWINDOW_H
#define CONSOLEWINDOW_H

#include <iostream>
// #include<arpa/inet.h>
// #include<unistd.h>
// #include<sys/socket.h>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
// #include "ckobuki.h"
// #include "rplidar.h"

#include "robot.h"

namespace Ui
{
    class ConsoleWindow;
}

class ConsoleWindow
{

public:
    // explicit ConsoleWindow();
    // ~ConsoleWindow();
    ConsoleWindow();
    ~ConsoleWindow();
    int processThisLidar(LaserMeasurement laserData);
    int processThisRobot(TKobukiData robotdata);
    void start_robot_smile();

private:
    //--skuste tu nic nevymazat... pridavajte co chcete, ale pri odoberani by sa mohol stat nejaky drobny problem, co bude vyhadzovat chyby
    Ui::ConsoleWindow *ui;
    // void paintEvent(QPaintEvent *event); // Q_DECL_OVERRIDE;
    int updateLaserPicture;
    LaserMeasurement copyOfLaserData;
    std::string ipaddress;
    Robot robot;
    TKobukiData robotdata;
    int datacounter;

    int sockfd;
    struct sockaddr_in servaddr;

    int sockfd_robot;
    struct sockaddr_in servaddr_robot;

    double forwardspeed;  // mm/s
    double rotationspeed; // omega/s
};

#endif