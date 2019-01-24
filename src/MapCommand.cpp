#include <ros/ros.h>
#include <thread>
#include <HDMap/srv_map_cmd.h>
#include <HDMap/msg_route_info.h>

using namespace std;

static std::string brief =
        "\n- - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n"
        "          |  |                                            \n"
        "         r43 r42                                          \n"
        "          |  |                                            \n"
        "  (j10) <---r1---- (j0) <---r9---- (j3) <---r11----       \n"
        "        ----r0--->      ----r8--->      ----r10---> |  |  \n"
        "   |  |            |  |            |  |             |  |  \n"
        "  r33 r32          r2 r3           r7 r6           r10 r11\n"
        "   |  |            |  |            |  |             |  |  \n"
        "   (j9) <---r27--- (j1) <---r5---- (j2) <---r13---- (j4)  \n"
        "   |    ----r26-->      ----r4--->      ----r12--->       \n"
        "   |  |            |  |            |  |             |  |  \n"
        "   | r35          r24 r25         r15 r14          r16 r17\n"
        "   |  |            |  |            |  |             |  |  \n"
        "   |(j8)<---r23--- (j7) <---r21--- (j6) <---r19---- (j5)  \n"
        "  r36   ----r22-->      ----r20-->      ----r18--->       \n"
        "   |  |            |  |            |  |                   \n"
        "   | r34          r28 r29         r31 r30     r41         \n"
        "   |  |            |  |            |  |                   \n"
        "  (j11) <---r38--- (j12)<---r40--- (j13)                  \n"
        "        ----r37-->      ----r39-->                        \n"
        "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - \n"
        "\e[1;32m[ r2r rid rid ]            road to road           \n"
        "\e[1;32m[ p2p x0 y0 d0 x1 y1 d1]   point to point         \n"
        "[    clear    ]                    clear exist route      \n"
        "[   exit / q  ]                    exit\e[0m              \n";

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
        std::cout << "] Tips: use 'r2r' or 'p2p 'to plan a route";
    else
        std::cout << "] Tips: use 'clear' to clear the current route";
    std::cout << "\e[0m\n";

    std::cout <<"$: " << std::flush;
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


void OnRouteInfoUpdate(const HDMap::msg_route_info& msg)
{
    Print(msg.route_id);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_command");
    ros::NodeHandle n;

    ros::ServiceClient client = n.serviceClient<HDMap::srv_map_cmd>("/map_command");
    ros::Subscriber  subscriber = n.subscribe("/map_pub_route_info",1,OnRouteInfoUpdate);

    auto command = [&]()
    {
        while(ros::ok())
        {
            string s;
            getline(cin, s);
            auto vec = split(s, " ");
            if (vec.empty())
            {
                std::cout << "invalid input\n";
                Print({});
            }
            else if(vec.front() == "r2r")
            {
                HDMap::srv_map_cmd srv;
                if(vec.size() < 3)
                {
                    std::cout << "invalid input" << std::endl;
                    Print({});
                    continue;
                }

                srv.request.cmd = vec.front();

                for(int i = 1; i < vec.size(); ++i){
                    srv.request.argv.emplace_back(stoi(vec[i]));
                }

                if(!client.call(srv))
                {
                    std::cout << "request service fail" << std::endl;
                    Print({});
                }
            }
            else if(vec.front() == "p2p")
            {
                HDMap::srv_map_cmd srv;
                if(vec.size() != 7)
                {
                    std::cout << "\ninvalid input\n\n";
                    Print({});
                    continue;
                }

                srv.request.cmd = vec.front();

                for(int i = 1; i < vec.size(); ++i){
                    srv.request.argv.emplace_back(stod(vec[i]));
                }

                if(!client.call(srv))
                {
                    std::cout << "request service fail" << std::endl;
                    Print({});
                }
            }
            else if(vec.front() == "clear")
            {
                HDMap::srv_map_cmd srv;
                srv.request.cmd = vec.front();
                client.call(srv);
                Print({});
            }
            else if(vec.front() == "q" || vec.front() == "exit")
            {
                break;
            }
            else
            {
                std::cout << "invalid input\n";
                Print({});
            }
        }
    };
    
    Print({});
    
    std::thread command_thread(command);
    
    ros::spin();
    
    command_thread.join();
}