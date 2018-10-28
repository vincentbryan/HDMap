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
        tree.put("test.filename", "test");
        std::stringstream ss;
        pt::write_xml(ss, tree);
        string s = ss.str();
        std::cout << s << endl;
        std::cout << "\nSuccess\n";
    }
    catch (std::exception &e)
    {
        std::cout << "Error: " << e.what() << "\n";
    }
    return 0;
}