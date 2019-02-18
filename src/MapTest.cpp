// MapTest 提供了模拟位置发送、请求信息发送等示例，可以通过命令行进行简单操作
// 更多的细节参阅 doc/HDMap接口文档

#include <ros/ros.h>
#include <Tool/Sender.h>
#include <termios.h>
#include <HDMap/srv_map_data.h>
#include <HDMap/msg_route_region.h>
#include <HDMap/srv_route.h>
#include <tf/transform_datatypes.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <nav_msgs/Odometry.h>

using namespace std;

vector<string> split(const string & str, const string & pattern)
{
    //const char* convert to char*
    char * strc = new char[strlen(str.c_str())+1];
    strcpy(strc, str.c_str());
    vector<string> resultVec;
    char* tmpStr = strtok(strc, pattern.c_str());
    while (tmpStr != nullptr)
    {
        resultVec.emplace_back(string(tmpStr));
        tmpStr = strtok(nullptr, pattern.c_str());
    }
    delete[] strc;

    return resultVec;
}

void CommandPrint()
{
    printf("\e[1;33m$: ");
}

int getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
    int c = getchar();  // read character (non-blocking)
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}

void renderCar(ros::Publisher& MarkerPublisher, double x, double y, double angle)
{

    visualization_msgs::Marker carmarker;
    carmarker.header.frame_id = "map";
    carmarker.header.stamp = ros::Time();
    carmarker.ns = "map_test";
    carmarker.id = 0;
    carmarker.type = visualization_msgs::Marker::MESH_RESOURCE;
    carmarker.action = visualization_msgs::Marker::ADD;
    carmarker.pose.position.x = x;
    carmarker.pose.position.y = y;
    carmarker.pose.position.z = 0.7;
    carmarker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(TFSIMD_HALF_PI,0,TFSIMD_HALF_PI+angle);
    carmarker.scale.x = 1;
    carmarker.scale.y = 1;
    carmarker.scale.z = 1;
    carmarker.color.a = carmarker.color.r = carmarker.color.b = carmarker.color.g = 1;
    carmarker.mesh_resource = "package://HDMap/res/car_texture/car.dae";
    carmarker.mesh_use_embedded_materials = 1;

    visualization_msgs::Marker textmarker;

    char _buf[32];
    sprintf(_buf,"(%3.2f, %3.2f, %3.1f)",x,y,angle*180.0/TFSIMD_PI);
    textmarker.text= _buf;

    textmarker.header.frame_id = "map";
    textmarker.header.stamp = ros::Time();
    textmarker.ns = "map_test";
    textmarker.id = 1;
    textmarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textmarker.action = visualization_msgs::Marker::ADD;
    textmarker.pose.position.x = x;
    textmarker.pose.position.y = y;
    textmarker.pose.position.z = 5;
    textmarker.scale.x = 1;
    textmarker.scale.y = 1;
    textmarker.scale.z = 1;
    textmarker.color.a = 1;
    textmarker.color.b = 1;
    textmarker.color.g = 1;
    textmarker.color.r = 1;

    visualization_msgs::MarkerArray array;
    array.markers.emplace_back(carmarker);
    array.markers.emplace_back(textmarker);


    MarkerPublisher.publish(array);

}

void renderPolygon(const HDMap::msg_route_region& msg)
{
    std::cout << "----------------------------------\n";
    for(auto & polygon:msg.polygons){
        std::cout <<"  "<< polygon.points.size() <<"  " << endl;
    }
    std::cout << "----------------------------------\n";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_test");
    ros::NodeHandle n;

    ros::Publisher GPSPublisher = n.advertise<nav_msgs::Odometry>("/odom", 1);
    ros::Publisher MarkerPublisher = n.advertise<visualization_msgs::MarkerArray>("/CurCar", 0);

    ros::ServiceClient map_data_client = n.serviceClient<HDMap::srv_map_data>("/map_data_service");
    ros::ServiceClient map_plan_client = n.serviceClient<HDMap::srv_route>("/map_plan_service");
    ros::Subscriber map_sub_route_region = n.subscribe("/map_pub_route_region",10,renderPolygon);

    // mode values
    bool is_cut_commond_IO = false;
    double cur_x,cur_y,cur_d,pwm=1;

    while(ros::ok()){

        if(!is_cut_commond_IO){
            CommandPrint();
            string s;
            getline(cin, s);
            auto vec = split(s, " ");
            if (vec.front() == "gps" && vec.size()>=3 && vec.size()<=4){

                nav_msgs::Odometry _odom;
                _odom.header.stamp = ros::Time::now();
                cur_x = _odom.pose.pose.position.x  =   std::stod(vec[1]);
                cur_y = _odom.pose.pose.position.y  =   std::stod(vec[2]);
                cur_d = vec.size()==4? std::stod(vec[3]):0;

                _odom.pose.pose.orientation =  tf::createQuaternionMsgFromYaw( (cur_d - 90) * TFSIMD_PI/180.0 );

                GPSPublisher.publish(_odom);

                renderCar(MarkerPublisher,cur_x, cur_y, cur_d*TFSIMD_PI/180.0);
            }
            else if (vec.front()=="control")
            {
                std::cout << "Start Control Mode\n";
                is_cut_commond_IO = true;
                pwm=1;
            }
            else if(vec.front()=="data")
            {
                string type = vec[1];
                if(((type=="JunctionByPos" || type == "RoadByPos")&&vec.size()<4))
                {
                    std::cout << "invalid input\n\n";
                    continue;
                }

                if(type!="JunctionByPos" && type != "RoadByPos" &&type!="JunctionById" && type != "RoadById" )
                {
                    std::cout << "invalid input\n\n";
                    continue;
                }


                HDMap::srv_map_data srv_map_data;
                srv_map_data.request.type=type;
                for(int i = 2; i < vec.size(); ++i)
                {
                    srv_map_data.request.argv.emplace_back(std::strtod(vec[i].c_str(), nullptr));
                }

                if(map_data_client.call(srv_map_data)){
                    std::cout <<"\n"<< srv_map_data.response.res << endl;
                }
                else{
                    std::cerr << "Server error occurred\n\n";
                }

            }
            else if (vec.front() == "plan" && vec.size() >= 4)
            {
                HDMap::srv_route srv_route;
                srv_route.request.method = vec[1];
                for(int i = 2; i < vec.size(); ++i)
                {
                     srv_route.request.argv.emplace_back(std::stod(vec[i]));
                }

                if (map_plan_client.call(srv_route))
                {
                    std::stringstream ss;
                    ss << srv_route.response.route;
                    boost::property_tree::ptree tree;
                    boost::property_tree::read_xml(ss, tree);

                    cout << "Roads:\n";
                    for (auto &r: tree.get_child("hdmap.roads")) {
                        std::cout << r.second.get<int>("<xmlattr>.id") << " ";
                    }
                    cout <<"\nJunctions:\n";
                    for (auto &r: tree.get_child("hdmap.junctions")) {
                        std::cout << r.second.get<int>("<xmlattr>.id") << " ";
                    }
                    cout <<"\n";
                }
            }
            else{
                std::cout << "invalid input\n\n";
            }
        }
        else{
            int val = getch();
            if(val == 'q'){
                is_cut_commond_IO = false;
                std::cout << "End Control Mode\n\n";
            }
            else{
                printf("\b");
                double d = 0;
                switch (val){
                    case 'w':{
                        d = pwm;
                    }break;
                    case 's':{
                        d = -pwm;
                    }break;
                    case 'a':{
                        cur_d = fmod (cur_d + pwm,360);
                    }break;
                    case 'd':{
                        cur_d = fmod (cur_d - pwm,360);
                    }break;
                    case ' ':{
                        if (pwm<10)++pwm;
                    }break;
                    default:{
                        if (pwm>1)--pwm;
                    }
                }

                double _rad_cur_d = cur_d*TFSIMD_PI/180;
                cur_x = cos(_rad_cur_d)*d+cur_x;
                cur_y = sin(_rad_cur_d)*d+cur_y;

                nav_msgs::Odometry _odom;
                _odom.header.stamp = ros::Time::now();
                _odom.pose.pose.position.x = cur_x;
                _odom.pose.pose.position.y = cur_y;
                _odom.pose.pose.orientation =  tf::createQuaternionMsgFromYaw(_rad_cur_d - TFSIMD_HALF_PI);

                GPSPublisher.publish(_odom);
                renderCar(MarkerPublisher,cur_x, cur_y, _rad_cur_d);
            }
        }

        ros::spinOnce();

    }

}