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
#include <thread>
#include <atomic>
#include <chrono>

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
    unsigned short lastLeftEncoder = 0, lastRightEncoder = 0;
    ConsoleWindow();
    ~ConsoleWindow();
    int processThisLidar(LaserMeasurement laserData);
    int processThisRobot(TKobukiData robotdata);
    void start_robot_smile();

    void startOdomThread();
    void startMovementThread();
    void stopThreads();

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

    bool isMoving = true;

    std::atomic<bool> running;
    void odomThreadFunc();
    void movementThreadFunc();

    int sockfd;
    struct sockaddr_in servaddr;

    int sockfd_robot;
    struct sockaddr_in servaddr_robot;

    double forwardspeed;  // mm/s
    double rotationspeed; // omega/s

    std::thread odomThread;
    std::thread movementThread;
};

#endif