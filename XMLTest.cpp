//
// Created by vincent on 18-10-13.
//
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <set>
#include <exception>
#include <iostream>
namespace pt = boost::property_tree;

using namespace std;

int main()
{
    try
    {
        pt::ptree tree;
        pt::read_xml("test.xml", tree);

        for(auto road : tree.get_child("hdmap.roads"))
        {
            cout << road.second.get<int>("<xmlattr>.id") << " " << road.second.get<double>("<xmlattr>.length") << endl;
            for(auto section : road.second.get_child(""))
            {
                if(section.first == "lanesection")
                for(auto section_child : section.second.get_child(""))
                {
                    cout << section_child.first << endl;
                }
            }

//            for(auto section : road.second.get_child("lanesection"))
//            {
//                if(section.first == "<xmlattr>")
//                    std::cout << section.second.get<int>("id") << " "  << section.second.get<double>("s") << std::endl;
//                if(section.first == "refer_line" || section.first == "offset")
//                {
//                    std::cout << section.second.get<string>("<xmlattr>.type") << " "
//                              << section.second.get<double>("<xmlattr>.s") << endl;
//                    std::cout << section.second.get<double>("start_pose.x") << " "
//                              << section.second.get<double>("start_pose.y") << " "
//                              << section.second.get<double>("start_pose.yaw") << "";
//                    std::cout << section.second.get<double>("end_pose.x") << " "
//                              << section.second.get<double>("end_pose.y") << " "
//                              << section.second.get<double>("end_pose.yaw") << endl;
//                }
//                if(section.first == "lane")
//                {
//                    cout << section.second.get<int>("<xmlattr>.idx") << " "
//                         << section.second.get<int>("<xmlattr>.id") << " "
//                         << section.second.get<string>("<xmlattr>.type") << " ";
//                    std::cout << section.second.get<double>("offset.start_pose.x") << " "
//                              << section.second.get<double>("offset.start_pose.y") << " "
//                              << section.second.get<double>("offset.start_pose.yaw") << "";
//                    std::cout << section.second.get<double>("offset.end_pose.x") << " "
//                              << section.second.get<double>("offset.end_pose.y") << " "
//                              << section.second.get<double>("offset.end_pose.yaw") << endl;
//                }
//            }

        }

        std::cout << "\nSuccess\n";
    }
    catch (std::exception &e)
    {
        std::cout << "Error: " << e.what() << "\n";
    }
    return 0;
}