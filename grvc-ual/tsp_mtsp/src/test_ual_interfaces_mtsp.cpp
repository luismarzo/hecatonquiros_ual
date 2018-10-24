//----------------------------------------------------------------------------------------------------------------------
// GRVC UAL
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2016 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------

#include "mtsp_files/funcion_mtsp_file.h"
using namespace std;
using std::vector;

#include <uav_abstraction_layer/ual.h>
#include <ros/ros.h>
#include <fstream>
#include <string>
#include <iostream>

int main(int _argc, char **_argv)
{

    std::ofstream file;
    file.open("pos.txt");
    unsigned int FileLength = 0;

     string line;
     ifstream myfile("/home/luis/ws_ual/src/grvc-ual/tsp_mtsp/src/pos.txt", ios::in | ios::out | ios::binary);
     if (!myfile)
     {
         cout << "cannot open file";
         exit(1);
     }

     while (!myfile.eof())
     {
         getline(myfile, line);
         FileLength++;
     }
     std::cout << FileLength << std::endl;
     file.close();

    vector<vector<float>> salida, entrada, puntos_recorrido;

    grvc::ual::UAL ual(_argc, _argv);
    while (!ual.isReady() && ros::ok())
    {
        std::cout << "UAL not ready!" << std::endl;
        sleep(1);
    }

    // Start server if explicitly asked:
    ros::NodeHandle pnh("~");
    std::string server_mode;
    pnh.param<std::string>("ual_server", server_mode, "off");
    if (server_mode == "on")
    {
        std::cout << "UAL server is now available" << std::endl;
        // Do nothing
        while (ros::ok())
        {
            sleep(1);
        }
        return 0;
    }

    // Else, use class interface to perform a simple mission:
    std::cout << "Mission using UAL object" << std::endl;
    // Define flight level and take off
    double flight_level = 15.0;
    ual.takeOff(flight_level);

    /* //CIUDADES POR LAS QUE PASAR

    entrada.resize(7);
               for(int i=0;i<entrada.size();i++){
              entrada[i].resize(3); // COORDENADAS 3D
              }

    //LLENAR (ENTRADA)
           entrada[0][0]=1; entrada[0][1]=2; entrada[0][2]=flight_level;
           entrada[1][0]=4; entrada[1][1]=5; entrada[1][2]=flight_level;
           entrada[2][0]=7; entrada[2][1]=8; entrada[2][2]=flight_level;
           entrada[3][0]=3; entrada[3][1]=3; entrada[3][2]=flight_level;
           entrada[4][0]=1; entrada[4][1]=-4; entrada[4][2]=flight_level;
           entrada[5][0]=-1; entrada[5][1]=-1; entrada[5][2]=flight_level;

           entrada[6][0]=13;   entrada[6][1]=4;    entrada[6][2]=flight_level;


    // FUNCIÓN DEL MTSP
      salida=funcion_mtsp(entrada);
*/



    // Define path (ugly due to lack of constructor)
    double square_lenght = 10.0;
    std::list<grvc::ual::Waypoint> path;
    grvc::ual::Waypoint waypoint;

    for (int i = 0; i < 7; i++)
    {
        waypoint.header.frame_id = "map";
        waypoint.pose.position.x = salida[i][0];
        waypoint.pose.position.y = salida[i][1];
        waypoint.pose.position.z = flight_level;
        waypoint.pose.orientation.x = 0;
        waypoint.pose.orientation.y = 0;
        waypoint.pose.orientation.z = 0;
        waypoint.pose.orientation.w = 1;
        path.push_back(waypoint);
    }

    std::cout << "Blocking version of goToWaypoint" << std::endl;
    for (auto p : path)
    {
        std::cout << "Waypoint: " << p.pose.position.x << ", " << p.pose.position.y << ", " << p.pose.position.z << ", frame_id: " << p.header.frame_id << std::endl;
        ual.goToWaypoint(p);
        std::cout << "Arrived!" << std::endl;
    }

    std::cout << "Non-blocking version of goToWaypoint" << std::endl;
    for (auto p : path)
    {
        std::cout << "Waypoint: " << p.pose.position.x << ", " << p.pose.position.y << ", " << p.pose.position.z << ", frame_id: " << p.header.frame_id << std::endl;
        ual.goToWaypoint(p, false);
        do
        {
            grvc::ual::Velocity v = ual.velocity();
            std::cout << "v = [" << v.twist.linear.x << ", " << v.twist.linear.y << ", "
                      << v.twist.linear.z << "]" << std::endl;
            std::cout << "Waiting" << std::endl;
            sleep(1);
        } while (!ual.isIdle());
        std::cout << "Arrived!" << std::endl;
    }

    // Land
    ual.land();

    return 0;
}
