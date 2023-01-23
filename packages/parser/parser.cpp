#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include "json.hpp"
#include <math.h>

using json = nlohmann::json;
using namespace std;

struct stampStrc{
    public:
        int secs = 0;
        int nsecs = 0;
};

struct headerStrct{
    public:
        std::string frame_id;
        int seq = 0;
        stampStrc stamp;
};

struct coordinatesCalculadet{
    public:
        std::vector<double> x;
        std::vector<double> y;
};

struct msgLIDAR{
    public:
        headerStrct header;
        double angle_increment = 0;
        double angle_max = 0;
        double angle_min = 0;
        double time_increment = 0;
        double scan_time = 0;
        double range_max;
        double range_min;
        coordinatesCalculadet XYcoordinates;
        std::vector<double> intensities;
        std::vector<double> ranges;

};

struct posiCoordinates{
    public:
        double x = 0;
        double y = 0;
        double z = 0;
};

struct oriCoordinates{
    public:
        double x = 0;
        double y = 0;
        double z = 0;
        double w = 0;
};

struct pose{
    public:
        posiCoordinates position;
        oriCoordinates orientation;
        vector<double> covariance;
};

struct msgOddomtr{
    public:
        headerStrct header;
        string child_frame_id;
        pose pose;
        
};


int main()
{
    // the Message from the Turtlebot as a JSON txt
    auto LIDAR_input = R"(
    {
        "header": {"seq": 20326, "stamp": {"secs": 1667560518, "nsecs": 808402897}, "frame_id": "base_scan"}, "angle_min": 0.0, "angle_max": 6.2657318115234375, "angle_increment": 0.01745329238474369, "time_increment": 0.0005592841189354658, "scan_time": 0.20134228467941284, "range_min": 0.11999999731779099, "range_max": 3.5, "ranges": [ 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.00], "intensities": [63.0, 71.0, 60.0, 697.0, 0.0, 917.0, 869.0, 883.0, 894.0, 878.0, 859.0, 815.0, 811.0, 808.0, 820.0, 718.0, 371.0, 475.0, 572.0, 625.0, 843.0, 985.0, 1136.0, 1278.0, 1419.0, 1535.0, 1673.0, 1595.0, 2054.0, 1716.0, 104.0, 104.0, 118.0, 101.0, 106.0, 106.0, 1068.0, 1244.0, 148.0, 152.0, 176.0, 97.0, 638.0, 97.0, 167.0, 217.0, 137.0, 117.0, 200.0, 86.0, 81.0, 308.0, 603.0, 293.0, 461.0, 387.0, 304.0, 388.0, 419.0, 234.0, 79.0, 268.0, 193.0, 148.0, 116.0, 0.0, 82.0, 191.0, 191.0, 249.0, 187.0, 301.0, 373.0, 490.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 339.0, 75.0, 0.0, 313.0, 60.0, 0.0, 0.0, 0.0, 44.0, 79.0, 130.0, 0.0, 0.0, 0.0, 277.0, 336.0, 279.0, 0.0, 122.0, 605.0, 1471.0, 118.0, 64.0, 0.0, 45.0, 0.0, 0.0, 0.0, 0.0, 0.0, 33.0, 0.0, 43.0, 43.0, 40.0, 42.0, 39.0, 39.0, 0.0, 139.0, 279.0, 0.0, 244.0, 0.0, 438.0, 200.0, 0.0, 43.0, 0.0, 0.0, 2426.0, 0.0, 0.0, 36.0, 80.0, 0.0, 0.0, 0.0, 0.0, 82.0, 0.0, 0.0, 111.0, 38.0, 0.0, 842.0, 0.0, 2744.0, 74.0, 64.0, 46.0, 0.0, 0.0, 0.0, 0.0, 2251.0, 356.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 127.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 101.0, 66.0, 0.0, 112.0, 0.0, 0.0, 0.0, 0.0, 32.0, 0.0, 0.0, 0.0, 845.0, 40.0, 35.0, 54.0, 54.0, 0.0, 32.0, 41.0, 76.0, 93.0, 0.0, 0.0, 0.0, 0.0, 48.0, 124.0, 124.0, 0.0, 41.0, 52.0, 59.0, 68.0, 77.0, 65.0]
    }
    )";
    auto Odomtr_input = R"(
    {
        "header": {"seq": 102921, "stamp": {"secs": 1667560543, "nsecs": 435509935}, "frame_id": "odom"}, "child_frame_id": "base_footprint", "pose": {"pose": {"position": {"x": 8.138175964355469, "y": 0.07175924628973007, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0031610834412276745, "w": 0.9999949932098389}}, "covariance": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}, "twist": {"twist": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": -0.00023186545877251774}}, "covariance": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}

    }
    )";

    // new Msg struct to save the Turtlebot structured Message
    msgLIDAR msgL;
    msgOddomtr msgO;

    // parse the Message from the Turtlebot
    json j_msg_L = json::parse(LIDAR_input);
    json j_msg_O = json::parse(Odomtr_input);

    // feed the JSON File into the 'message' Struct
    j_msg_L["header"]["seq"].get_to(msgL.header.seq);
    j_msg_L["header"]["stamp"]["secs"].get_to(msgL.header.stamp.secs);
    j_msg_L["header"]["stamp"]["nsecs"].get_to(msgL.header.stamp.nsecs);
    j_msg_L["header"]["frame_id"].get_to(msgL.header.frame_id);
    j_msg_L["angle_increment"].get_to(msgL.angle_increment);
    j_msg_L["angle_max"].get_to(msgL.angle_max);
    j_msg_L["angle_min"].get_to(msgL.angle_min);
    j_msg_L["time_increment"].get_to(msgL.time_increment);
    j_msg_L["scan_time"].get_to(msgL.scan_time);
    j_msg_L["range_max"].get_to(msgL.range_max);
    j_msg_L["range_min"].get_to(msgL.range_min);
    j_msg_L["intensities"].get_to(msgL.intensities);
    j_msg_L["ranges"].get_to(msgL.ranges);

    j_msg_O["header"]["seq"].get_to(msgO.header.seq);
    j_msg_O["header"]["stamp"]["secs"].get_to(msgO.header.stamp.secs);
    j_msg_O["header"]["stamp"]["nsecs"].get_to(msgO.header.stamp.nsecs);
    j_msg_O["header"]["frame_id"].get_to(msgO.header.frame_id);
    j_msg_O["child_frame_id"].get_to(msgO.child_frame_id);
    j_msg_O["pose"]["pose"]["position"]["x"].get_to(msgO.pose.position.x);
    j_msg_O["pose"]["pose"]["position"]["y"].get_to(msgO.pose.position.y);
    j_msg_O["pose"]["pose"]["position"]["z"].get_to(msgO.pose.position.z);
    j_msg_O["pose"]["pose"]["orientation"]["x"].get_to(msgO.pose.orientation.x);
    j_msg_O["pose"]["pose"]["orientation"]["x"].get_to(msgO.pose.orientation.y);
    j_msg_O["pose"]["pose"]["orientation"]["x"].get_to(msgO.pose.orientation.z);
    j_msg_O["pose"]["pose"]["orientation"]["x"].get_to(msgO.pose.orientation.w);
    j_msg_O["pose"]["covariance"].get_to(msgO.pose.covariance);


    for(int i = 0; i < 360; i++)
    {
        msgL.XYcoordinates.x.push_back(msgL.ranges.at(i) * sin(msgL.angle_min + msgL.angle_increment * i));
        msgL.XYcoordinates.y.push_back(msgL.ranges.at(i) * cos(msgL.angle_min + msgL.angle_increment * i));
        
        if(i % 10 == 0)
        {
        std::cout << std::fixed << std::setprecision(0) << "(" << msgL.XYcoordinates.x.at(i) << " - " << msgL.XYcoordinates.y.at(i) << ")\n";
        }
    
    }


    
    

    // Test print:
    /*
            std::cout << std::endl << std::endl 
                << "Scan Id: \t\t" << msgL.header.frame_id << std::endl
                << "Seq. Nr: \t\t" << msgL.header.seq << std::endl 
                << "\t Sek.: \t\t" << msgL.header.stamp.secs << std::endl 
                << "\t nano Sek.: \t" << msgL.header.stamp.nsecs<< std::endl 
                << "Angle min: \t" << msgL.angle_min <<std::endl
                << "Angle max: \t" << msgL.angle_max <<std::endl
                << "Angle increment:" << msgL.angle_increment <<std::endl
                << "Range min: \t" << msgL.range_min << std::endl
                << "Range max: \t" << msgL.range_max << std::endl;
                std::cout << "First 36 Ranges:" << std::endl << "\t"; 
                
                for (int i = 0; i < 36; i++){
                     std::cout << i << ":\t" << msgL.ranges.at(i) << "\n "; 
                }
                */
                std::cout << std::endl << std::endl;

    return 0;
}


