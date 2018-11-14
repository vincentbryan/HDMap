//
// Created by vincent on 18-11-7.
//

#include <ros/ros.h>
#include <HDMap/srv_map_cmd.h>
using namespace std;

static std::string brief =
        "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n"
        "   (j10)<---r1---- (j0) <---r9---- (j3) <---r11----       \n"
        "        ----r0--->      ----r8--->      ----r10--->       \n"
        "   |  |            |  |            |  |             |  |  \n"
        "  r33 r32          r2 r3           r7 r6           r10 r11\n"
        "   |  |            |  |            |  |             |  |  \n"
        "   (j9) <---r27--- (j1) <---r5---- (j2) <---r13---- (j4)  \n"
        "        ----r26-->      ----r4--->      ----r21--->       \n"
        "   |  |            |  |            |  |             |  |  \n"
        "   | r36          r24 r25         r15 r14          r16 r17\n"
        "   |  |            |  |            |  |             |  |  \n"
        "   |(j8)<---r23--- (j7) <---r21--- (j6) <---r19---- (j5)  \n"
        "  r35   ----r22-->      ----r20-->      ----r18--->       \n"
        "   |  |            |  |            |  |                   \n"
        "   | r34          r28 r29         r31 r30     r41         \n"
        "   |  |            |  |            |  |                   \n"
        "   (j11)----r38--> (j12)----r40--> (j13)                  \n"
        "        <---r37---      <---r39---                        \n"
        "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n"
        "\e[1;32m[start rid rid]: start a route                    \n"
        "[     end     ]: finish a route                           \n"
        "[   exit / q  ]: exit\e[0m                                \n";

void Print(std::vector<int> v)
{
    system("clear");
    std::cout << brief;

    std::cout << "\e[1;33mRoute: [";
    if(v.empty())
        std::cout << "empty";
    else
        for(auto & x : v) std::cout << x << " ";

    if(v.empty())
        std::cout << "]\tTips: use 'start' to start a route";
    else
        std::cout << "]\tTips: use 'end' to finish the current route";
    std::cout << "\e[0m\n";

    std::cout << "$: ";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_command");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<HDMap::srv_map_cmd>("map_command");
    std::vector<int> v;
    while(ros::ok())
    {
        Print(v);
        string s;
        cin >> s;
        if(s == "start")
        {
            HDMap::srv_map_cmd srv;
            int start_rid, end_rid;
            cin >> start_rid >> end_rid;
            srv.request.cmd = "start";
            srv.request.argv1 = start_rid;
            srv.request.argv2 = end_rid;
            if(client.call(srv))
            {
                v = srv.response.route;
            }
        }
        else if(s == "end")
        {
            HDMap::srv_map_cmd srv;
            srv.request.cmd == "end";
            srv.request.argv1 = srv.request.argv2 = -1;
            client.call(srv);
            v.clear();
        }
        else if(s == "q" || s == "exit")
        {
            break;
        }
        else
        {
            std::cout << "invalid input\n";
        }
    }
}