#include "robot.h"
#include "consolewindow.h"
#include <unistd.h>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>

struct LidarData
{
    double distances[277];
    double angles[277];
};

struct OdomData
{
    double x;
    double y;
    double theta;
};

int ConsoleWindow::processThisLidar(LaserMeasurement laserData)
{

    LidarData data;
    for (int i = 0; i < laserData.numberOfScans; i++)
    {
        data.distances[i] = laserData.Data[i].scanDistance / 1000;
        data.angles[i] = laserData.Data[i].scanAngle;
        std::string message = std::to_string(i + 1) + " distance: " + std::to_string(laserData.Data[i].scanDistance / 1000.0f) + " angle: " + std::to_string(laserData.Data[i].scanAngle) + "\n";
        // std::cout << message;
    }
    if (sendto(sockfd, &data, sizeof(data), MSG_CONFIRM, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        std::cout << "Error sending data" << std::endl;
    }
    else
    {
        std::cout << "Lidar data sent" << std::endl;
    }
    return 0;
};



int ConsoleWindow::processThisRobot(TKobukiData robotdata )
{
    // lastLeftEncoderCopy = lastLeftEncoder;
    // lastRightEncoderCopy = lastRightEncoder;
    OdomData odomData;
    static unsigned short lastLeftEncoder = 0, lastRightEncoder = 0;
    long double tickToMeter = 0.000085292090497737556558;
    long double b = 0.23;
    static unsigned short temp = 0;
    forwardspeed = 0;
    rotationspeed = 0;

    robot.setRotationSpeed(rotationspeed);
    robot.setTranslationSpeed(forwardspeed);
    robot.setArcSpeed(forwardspeed, forwardspeed / rotationspeed);
    robot.setTranslationSpeed(0);

    double deltaLeft = (robotdata.EncoderLeft - lastLeftEncoder) * tickToMeter;
    double deltaRight = (robotdata.EncoderRight - lastRightEncoder) * tickToMeter;

    lastLeftEncoder = robotdata.EncoderLeft;
    lastRightEncoder = robotdata.EncoderRight;

    double deltaTheta = (deltaRight - deltaLeft) / b;
    double deltaD = (deltaRight + deltaLeft) / 2.0;

    odomData.x = deltaD * cos(odomData.theta + deltaTheta / 2);
    odomData.y = deltaD * sin(odomData.theta + deltaTheta / 2);
    odomData.theta = deltaTheta;

    // std::cout << "encoder left:" << robotdata.EncoderLeft << std::endl;
    // std::cout << "encoder right:" << robotdata.EncoderRight << std::endl;

    // std::cout << "deltaD:" << deltaD << std::endl;
    // std::cout << "deltaTheta:" << deltaTheta << std::endl;

    std::cout << "x: " << odomData.x << std::endl;
    std::cout << "y: " << odomData.y << std::endl;
    std::cout << "theta: " << odomData.theta << std::endl;

    // std::cout << "x: " << deltaD * cos(odomData.theta + deltaTheta / 2) << std::endl;
    // std::cout << "y:" << deltaD * sin(odomData.theta + deltaTheta / 2) << std::endl;
    if (sendto(sockfd_robot, &odomData, sizeof(odomData), MSG_CONFIRM, (const struct sockaddr *)&servaddr_robot, sizeof(servaddr_robot)) < 0)
    {
        std::cout << "Error sending data" << std::endl;
    }
    else
    {
        std::cout << "Data sent" << std::endl;
        
    }

    // here is the test

    // sleep(2);
    // robot.setTranslationSpeed(200);
    // sleep(2);
    // robot.setTranslationSpeed(0);
    // sleep(2);
    // robot.setTranslationSpeed(-200);
    // sleep(2);
    // robot.setTranslationSpeed(0);
    
    

    return 0;
}

void ConsoleWindow::start_robot_smile()
{
    ipaddress = "127.0.0.1";
    // ipaddress = "192.168.1.11";
    robot.setLaserParameters(ipaddress, 52999, 5299, std::bind(&ConsoleWindow::processThisLidar, this, std::placeholders::_1));
    robot.setRobotParameters(ipaddress, 53000, 5300, std::bind(&ConsoleWindow::processThisRobot, this, std::placeholders::_1));

    robot.robotStart();
}

ConsoleWindow::ConsoleWindow()
{
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        std::cerr << "socket creation failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));

    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(7777);
    servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");

    if ((sockfd_robot = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        std::cerr << "socket creation failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    memset(&servaddr_robot, 0, sizeof(servaddr_robot));

    servaddr_robot.sin_family = AF_INET;
    servaddr_robot.sin_port = htons(7776);
    servaddr_robot.sin_addr.s_addr = inet_addr("127.0.0.1");
}

ConsoleWindow::~ConsoleWindow()
{
    close(sockfd);
}

int main()
{
    ConsoleWindow console;
    console.start_robot_smile();

    return 0;
}
// // g++ -std=c++11 -o consolewindow consolewindow.cpp robot.cpp rplidar.cpp CKobuki.cpp -lpthread `pkg-config --cflags --libs opencv4`

// #include "robot.h"
// #include "consolewindow.h"
// #include <iostream>
// #include <cstring> // For memset
// #include <unistd.h>
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <arpa/inet.h>

// struct LidarData
// {
//     double distances[277];
//     double angles[277];
// };

// struct OdomData
// {
//     double x;
//     double y;
//     double theta;
// };

// int ConsoleWindow::processThisLidar(LaserMeasurement laserData)
// {
//     LidarData data;
//     for (int i = 0; i < laserData.numberOfScans; i++)
//     {
//         data.distances[i] = laserData.Data[i].scanDistance / 1000.0;
//         data.angles[i] = laserData.Data[i].scanAngle;
//     }
//     if (sendto(sockfd, &data, sizeof(data), 0, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
//     {
//         std::cerr << "Error sending Lidar data" << std::endl;
//     }
//     return 0;
// };

// int ConsoleWindow::processThisRobot(TKobukiData robotdata)
// {
//     OdomData odomData;
//     static unsigned short lastLeftEncoder=0, lastRightEncoder=0; // Static to retain their values across function calls
//     const long double tickToMeter = 0.000085292090497737556558;
//     const long double b = 0.23;

//     forwardspeed = 0;
//     rotationspeed = 0;

//     robot.setRotationSpeed(rotationspeed);
//     robot.setTranslationSpeed(forwardspeed);
//     robot.setArcSpeed(forwardspeed, forwardspeed / rotationspeed);
//     robot.setTranslationSpeed(0);

//     double deltaLeft = (robotdata.EncoderLeft - lastLeftEncoder) * tickToMeter;
//     double deltaRight = (robotdata.EncoderRight - lastRightEncoder) * tickToMeter;

//     lastLeftEncoder = robotdata.EncoderLeft;
//     lastRightEncoder = robotdata.EncoderRight;

//     double deltaTheta = (deltaRight - deltaLeft) / b;
//     double deltaD = (deltaRight + deltaLeft) / 2.0;

//     odomData.x = deltaD * cos(odomData.theta + deltaTheta / 2);
//     odomData.y = deltaD * sin(odomData.theta + deltaTheta / 2);
//     odomData.theta = deltaTheta;

//     if (sendto(sockfd_robot, &odomData, sizeof(odomData), 0, (const struct sockaddr *)&servaddr_robot, sizeof(servaddr_robot)) < 0)
//     {
//         std::cerr << "Error sending Robot data" << std::endl;
//     }

//     return 0;
// }

// void ConsoleWindow::start_robot_smile()
// {
//     ipaddress = "192.168.1.11"; // Assuming this is the correct IP address
//     robot.setLaserParameters(ipaddress, 52999, 5299, std::bind(&ConsoleWindow::processThisLidar, this, std::placeholders::_1));
//     robot.setRobotParameters(ipaddress, 53000, 5300, std::bind(&ConsoleWindow::processThisRobot, this, std::placeholders::_1));

//     robot.robotStart();
// }

// ConsoleWindow::ConsoleWindow()
// {
//     if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
//     {
//         std::cerr << "Socket creation failed" << std::endl;
//         exit(EXIT_FAILURE);
//     }

//     memset(&servaddr, 0, sizeof(servaddr));
//     servaddr.sin_family = AF_INET;
//     servaddr.sin_port = htons(7777);
//     servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");

//     if ((sockfd_robot = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
//     {
//         std::cerr << "Socket creation failed" << std::endl;
//         exit(EXIT_FAILURE);
//     }

//     memset(&servaddr_robot, 0, sizeof(servaddr_robot));
//     servaddr_robot.sin_family = AF_INET;
//     servaddr_robot.sin_port = htons(7776);
//     servaddr_robot.sin_addr.s_addr = inet_addr("127.0.0.1");
// }

// ConsoleWindow::~ConsoleWindow()
// {
//     close(sockfd);
//     close(sockfd_robot);
// }

// int main()
// {
//     ConsoleWindow console;
//     console.start_robot_smile();

//     return 0;
// }
