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
    double x0, y0, x1, y1;
    while (cin >> x0 >> y0 >> x1 >> y1)
        std::cout << atan2(y1-y0, x1-x0) / M_PI * 180.0 << std::endl;
    /*
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
     */
    return 0;
}