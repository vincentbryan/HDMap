//
// Created by vincent on 18-11-7.
//

#include <ros/ros.h>
#include <HDMap/srv_map_cmd.h>
using namespace std;

static std::string brief =
        "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n"
        "  (j10) <---r1---- (j0) <---r9---- (j3) <---r11----       \n"
        "        ----r0--->      ----r8--->      ----r10--->       \n"
        "   |  |            |  |            |  |             |  |  \n"
        "  r33 r32          r2 r3           r7 r6           r10 r11\n"
        "   |  |            |  |            |  |             |  |  \n"
        "   (j9) <---r27--- (j1) <---r5---- (j2) <---r13---- (j4)  \n"
        "        ----r26-->      ----r4--->      ----r12--->       \n"
        "   |  |            |  |            |  |             |  |  \n"
        "   | r36          r24 r25         r15 r14          r16 r17\n"
        "   |  |            |  |            |  |             |  |  \n"
        "   |(j8)<---r23--- (j7) <---r21--- (j6) <---r19---- (j5)  \n"
        "  r35   ----r22-->      ----r20-->      ----r18--->       \n"
        "   |  |            |  |            |  |                   \n"
        "   | r34          r28 r29         r31 r30     r41         \n"
        "   |  |            |  |            |  |                   \n"
        "  (j11) ----r38--> (j12)----r40--> (j13)                  \n"
        "        <---r37---      <---r39---                        \n"
        "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n"
        "\e[1;32m[start rid rid]: start a route                    \n"
        "[ seq rid ... ]: start a route with a road sequence       \n"
        "[     end     ]: finish a route                           \n"
        "[   exit / q  ]: exit\e[0m                                \n";

void Print(std::vector<int> v)
{
    system("clear");
    std::cout << brief;

    std::cout << "\e[1;33mRoute: [ ";
    if(v.empty())
        std::cout << "empty ";
    else
        for(auto & x : v) std::cout << x << " ";

    if(v.empty())
        std::cout << "] Tips: use 'start' to start a route";
    else
        std::cout << "] Tips: use 'end' to finish the current route";
    std::cout << "\e[0m\n";

    std::cout << "$: ";
}

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
        getline(cin, s);
        auto vec = split(s, " ");
        if(vec.front() == "start")
        {
            HDMap::srv_map_cmd srv;
            if(vec.size() != 3)
            {
                std::cout << "invalid input\n";
                continue;
            }

            srv.request.cmd = "start";
            srv.request.argv.emplace_back(atoi(vec[1].c_str()));
            srv.request.argv.emplace_back(atoi(vec[2].c_str()));

            if(client.call(srv))
            {
                v = srv.response.route;
            }
        }
        else if(vec.front() == "seq")
        {
            HDMap::srv_map_cmd srv;
            if(vec.size() <= 1)
            {
                std::cout << "invalid input\n";
                continue;
            }

            srv.request.cmd = "seq";
            for(int i = 1; i < vec.size(); i++)
            {
                srv.request.argv.emplace_back(atoi(vec[i].c_str()));
            }

            if(client.call(srv))
            {
                v = srv.response.route;
            }

        }
        else if(vec.front() == "end")
        {
            HDMap::srv_map_cmd srv;
            srv.request.cmd == "end";

            client.call(srv);
            v.clear();
        }
        else if(vec.front() == "q" || vec.front() == "exit")
        {
            break;
        }
        else
        {
            std::cout << "invalid input\n";
        }
    }
}