#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include "json.hpp"

using json = nlohmann::json;
using namespace std;

// struchts to save the json massage
struct stamp{
    public:
        int secs = 0;
        int nsecs = 0;
};

struct header{
    public:
        string frame_id;
        int seq = 0;
        stamp stamp;
};

struct msgLIDAR{
    public:
        header header;
        double angle_increment = 0;
        double angle_max = 0;
        double angle_min = 0;
        double time_increment = 0;
        double scan_time = 0;
        double range_max;
        double range_min;
        vector<double> intensities;
        vector<double> ranges;

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
        header header;
        string child_frame_id;
        pose pose;
        
};


int main()
{
    // the Message from the Turtlebot as a JSON txt
    auto LIDAR_input = R"(
    {
        "header": {"seq": 20326, "stamp": {"secs": 1667560518, "nsecs": 808402897}, "frame_id": "base_scan"}, "angle_min": 0.0, "angle_max": 6.2657318115234375, "angle_increment": 0.01745329238474369, "time_increment": 0.0005592841189354658, "scan_time": 0.20134228467941284, "range_min": 0.11999999731779099, "range_max": 3.5, "ranges": [0.0, 0.0, 0.0, 2.805000066757202, 0.0, 2.006999969482422, 2.003999948501587, 2.010999917984009, 2.00600004196167, 2.0190000534057617, 2.0369999408721924, 2.0399999618530273, 2.0339999198913574, 2.0490000247955322, 2.0510001182556152, 2.055999994277954, 2.005000114440918, 1.8940000534057617, 1.8530000448226929, 1.7760000228881836, 1.6770000457763672, 1.6119999885559082, 1.5529999732971191, 1.4850000143051147, 1.4329999685287476, 1.3830000162124634, 1.3389999866485596, 1.2860000133514404, 1.2710000276565552, 1.2239999771118164, 1.1920000314712524, 1.1619999408721924, 1.0889999866485596, 1.093999981880188, 1.062999963760376, 1.0479999780654907, 1.00600004196167, 0.9589999914169312, 0.9589999914169312, 0.9390000104904175, 0.9210000038146973, 0.8999999761581421, 0.8889999985694885, 0.8799999952316284, 0.8730000257492065, 0.8619999885559082, 0.8489999771118164, 0.8330000042915344, 0.8209999799728394, 0.7870000004768372, 0.7739999890327454, 0.7720000147819519, 0.7639999985694885, 0.7549999952316284, 0.7549999952316284, 0.7419999837875366, 0.7289999723434448, 0.7250000238418579, 0.6970000267028809, 0.6819999814033508, 0.6869999766349792, 0.6940000057220459, 0.6850000023841858, 0.6919999718666077, 0.6919999718666077, 0.7009999752044678, 0.6959999799728394, 0.6899999976158142, 0.6880000233650208, 0.6850000023841858, 0.6809999942779541, 0.6769999861717224, 0.6729999780654907, 0.6690000295639038, 0.6650000214576721, 0.6620000004768372, 0.6589999794960022, 0.6549999713897705, 0.6549999713897705, 0.6520000100135803, 0.6520000100135803, 0.6499999761581421, 0.6489999890327454, 0.6480000019073486, 0.6489999890327454, 0.6480000019073486, 0.6449999809265137, 0.6439999938011169, 0.6499999761581421, 0.6489999890327454, 0.6499999761581421, 0.6510000228881836, 0.6510000228881836, 0.6520000100135803, 0.6539999842643738, 0.6549999713897705, 0.6549999713897705, 0.6589999794960022, 0.6610000133514404, 0.6639999747276306, 0.6660000085830688, 0.6690000295639038, 0.6729999780654907, 0.675000011920929, 0.6809999942779541, 0.6869999766349792, 0.6909999847412109, 0.6959999799728394, 0.7009999752044678, 0.7020000219345093, 0.6620000004768372, 0.6620000004768372, 0.6759999990463257, 0.6819999814033508, 0.7139999866485596, 0.7149999737739563, 0.7170000076293945, 0.7160000205039978, 0.7269999980926514, 0.7279999852180481, 0.7269999980926514, 0.7760000228881836, 0.7730000019073486, 0.7710000276565552, 0.777999997138977, 0.7879999876022339, 0.8050000071525574, 0.8240000009536743, 0.8349999785423279, 0.8550000190734863, 0.8600000143051147, 0.8700000047683716, 0.8859999775886536, 0.8960000276565552, 0.9279999732971191, 0.9449999928474426, 0.9539999961853027, 0.9710000157356262, 0.9860000014305115, 1.0149999856948853, 1.0520000457763672, 1.0640000104904175, 1.1069999933242798, 1.1269999742507935, 1.190000057220459, 1.1920000314712524, 1.2309999465942383, 1.2760000228881836, 1.3289999961853027, 1.3630000352859497, 1.3940000534057617, 1.440000057220459, 1.503000020980835, 1.5720000267028809, 1.6299999952316284, 1.7020000219345093, 1.7860000133514404, 1.815000057220459, 1.8020000457763672, 1.7829999923706055, 1.7710000276565552, 1.8760000467300415, 1.9490000009536743, 2.058000087738037, 2.806999921798706, 3.0299999713897705, 3.2139999866485596, 3.5490000247955322, 3.943000078201294, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.107999801635742, 2.7320001125335693, 0.0, 0.0, 0.0, 0.0, 3.9179999828338623, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.5429999828338623, 2.9070000648498535, 3.5969998836517334, 3.5950000286102295, 3.6089999675750732, 3.9140000343322754, 3.0889999866485596, 3.0920000076293945, 4.019000053405762, 0.0, 3.4509999752044678, 3.4179999828338623, 3.441999912261963, 0.0, 0.0, 0.0, 3.194000005722046, 3.1410000324249268, 3.069000005722046, 2.8420000076293945, 2.138000011444092, 2.1389999389648438, 2.302000045776367, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.4570000171661377, 0.0, 0.0, 3.434999942779541, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0439999103546143, 3.013000011444092, 3.010999917984009, 0.0, 0.0, 3.4800000190734863, 3.440000057220459, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.493000030517578, 0.0, 2.4000000953674316, 0.0, 1.621000051498413, 1.718000054359436, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.5799999237060547, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.073999881744385, 0.0, 0.0, 4.002999782562256, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "intensities": [63.0, 71.0, 60.0, 697.0, 0.0, 917.0, 869.0, 883.0, 894.0, 878.0, 859.0, 815.0, 811.0, 808.0, 820.0, 718.0, 371.0, 475.0, 572.0, 625.0, 843.0, 985.0, 1136.0, 1278.0, 1419.0, 1535.0, 1673.0, 1595.0, 2054.0, 1716.0, 1732.0, 1818.0, 1849.0, 2478.0, 2572.0, 3061.0, 2434.0, 2130.0, 2035.0, 2214.0, 1951.0, 2191.0, 1995.0, 2380.0, 2630.0, 2516.0, 2871.0, 2769.0, 2807.0, 2865.0, 2703.0, 2459.0, 2342.0, 2479.0, 2504.0, 2426.0, 2651.0, 2479.0, 2545.0, 2731.0, 2754.0, 2854.0, 2734.0, 2743.0, 3016.0, 3011.0, 3060.0, 3182.0, 3312.0, 3179.0, 3399.0, 3283.0, 2984.0, 3113.0, 3050.0, 2876.0, 3020.0, 2956.0, 2997.0, 2701.0, 3084.0, 2755.0, 3007.0, 2714.0, 3047.0, 3031.0, 3021.0, 2796.0, 2910.0, 2911.0, 2818.0, 2879.0, 2796.0, 3187.0, 2942.0, 2974.0, 2917.0, 3018.0, 2832.0, 2954.0, 3040.0, 3202.0, 3125.0, 3191.0, 3325.0, 3198.0, 3182.0, 3304.0, 3001.0, 2820.0, 2906.0, 2790.0, 2775.0, 2773.0, 2811.0, 2582.0, 2618.0, 2399.0, 2488.0, 2665.0, 2518.0, 2454.0, 2539.0, 2664.0, 3027.0, 2879.0, 2768.0, 2762.0, 2735.0, 2606.0, 2384.0, 2344.0, 2253.0, 2379.0, 2283.0, 2096.0, 2320.0, 2303.0, 2687.0, 2645.0, 2241.0, 2114.0, 2199.0, 1867.0, 1839.0, 1729.0, 1553.0, 1885.0, 1746.0, 1569.0, 1435.0, 1328.0, 1180.0, 1058.0, 934.0, 829.0, 648.0, 1276.0, 1292.0, 1282.0, 1524.0, 463.0, 389.0, 276.0, 186.0, 178.0, 172.0, 167.0, 127.0, 108.0, 69.0, 106.0, 102.0, 103.0, 104.0, 104.0, 118.0, 101.0, 106.0, 106.0, 1068.0, 1244.0, 148.0, 152.0, 176.0, 97.0, 638.0, 97.0, 167.0, 217.0, 137.0, 117.0, 200.0, 86.0, 81.0, 308.0, 603.0, 293.0, 461.0, 387.0, 304.0, 388.0, 419.0, 234.0, 79.0, 268.0, 193.0, 148.0, 116.0, 0.0, 82.0, 191.0, 191.0, 249.0, 187.0, 301.0, 373.0, 490.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 339.0, 75.0, 0.0, 313.0, 60.0, 0.0, 0.0, 0.0, 44.0, 79.0, 130.0, 0.0, 0.0, 0.0, 277.0, 336.0, 279.0, 0.0, 122.0, 605.0, 1471.0, 118.0, 64.0, 0.0, 45.0, 0.0, 0.0, 0.0, 0.0, 0.0, 33.0, 0.0, 43.0, 43.0, 40.0, 42.0, 39.0, 39.0, 0.0, 139.0, 279.0, 0.0, 244.0, 0.0, 438.0, 200.0, 0.0, 43.0, 0.0, 0.0, 2426.0, 0.0, 0.0, 36.0, 80.0, 0.0, 0.0, 0.0, 0.0, 82.0, 0.0, 0.0, 111.0, 38.0, 0.0, 842.0, 0.0, 2744.0, 74.0, 64.0, 46.0, 0.0, 0.0, 0.0, 0.0, 2251.0, 356.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 127.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 101.0, 66.0, 0.0, 112.0, 0.0, 0.0, 0.0, 0.0, 32.0, 0.0, 0.0, 0.0, 845.0, 40.0, 35.0, 54.0, 54.0, 0.0, 32.0, 41.0, 76.0, 93.0, 0.0, 0.0, 0.0, 0.0, 48.0, 124.0, 124.0, 0.0, 41.0, 52.0, 59.0, 68.0, 77.0, 65.0]
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
    /* Twist is not needed
    j_msg_O["twist"]["twist"]["linear"]["x"].get_to(msgO.twist.linear.x);
    j_msg_O["twist"]["twist"]["linear"]["y"].get_to(msgO.twist.linear.y);
    j_msg_O["twist"]["twist"]["linear"]["z"].get_to(msgO.twist.linear.z);
    j_msg_O["twist"]["twist"]["angular"]["x"].get_to(msgO.twist.angular.x);
    j_msg_O["twist"]["twist"]["angular"]["y"].get_to(msgO.twist.angular.y);
    j_msg_O["twist"]["twist"]["angular"]["z"].get_to(msgO.twist.angular.z);
    j_msg_O["twist"]["covariance"].get_to(msgO.twist.covariance);
    */

    // Test print:
    cout << "TEST PRINT:\n\nScan Id: \t\t" << msgL.header.frame_id << endl
    << "Seq. Nr: \t\t" << msgL.header.seq << endl 
    << "Sek.: \t\t\t" << msgL.header.stamp.secs << endl 
    << "Range at pos. 8: \t" << msgL.ranges.at(8) << endl 
    << "Intensities at pos. 9: \t" << msgL.intensities.at(9) << endl 
    << "Angle Increament: \t" << msgL.angle_increment << endl;

    cout << endl << endl;

    cout << "Scan Id: \t\t" << msgO.header.frame_id << endl
    << "Seq. Nr: \t\t" << msgO.header.seq << endl 
    << "Sek.: \t\t\t" << msgO.header.stamp.secs << endl 
    << "Covariance at pos. 8: \t" << msgO.pose.covariance.at(8) << endl 
    << "X-Coordinates: \t\t" << msgO.pose.orientation.x << endl;

    return 0;
}


