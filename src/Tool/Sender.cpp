#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <Tool/Sender.h>

using namespace hdmap;

unsigned int Sender::id = 0;

Sender::Sender(ros::Publisher pub_) : frame_id("/map"), pub(pub_)
{};

visualization_msgs::Marker Sender::GetLineStrip(std::vector<Pose> poses, double r, double g, double b, double a, double z, double width)
{
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = frame_id;
    line_strip.header.stamp = ros::Time::now();
    line_strip.id = id++;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    line_strip.scale.x = width;

    auto func = [](double x)
    {
        x = std::abs(x);
        if(x > 1)
            return x - std::floor(x);
        else
            return x;
    };
    line_strip.color.r = func(r);
    line_strip.color.g = func(g);
    line_strip.color.b = func(b);
    line_strip.color.a = func(a);

    for(auto p : poses)
    {
        geometry_msgs::Point point;
        point.x = p.x;
        point.y = p.y;
        point.z = z;
        line_strip.points.push_back(point);
    }

    return line_strip;
}

visualization_msgs::Marker
Sender::GetText(const std::string &content, Coor p, double r, double g, double b, double a, double z, double scale)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.id = id++;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker.scale.z = scale;
    marker.color.b = b;
    marker.color.g = g;
    marker.color.r = r;
    marker.color.a = a;

    geometry_msgs::Pose pose;
    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.position.z = z;
    marker.text= content;
    marker.pose=pose;

    return marker;
}

visualization_msgs::Marker Sender::GetCone(const Coor &v, double r, double g, double b, double a, double scale) const
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.id = id++;
    marker.type = visualization_msgs::Marker::ARROW;

    marker.scale.z = scale;
    marker.scale.x = scale;
    marker.scale.y = scale;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    geometry_msgs::Point start;
    geometry_msgs::Point end ;
    start.x = v.x;
    start.y = v.y;
    start.z = 0.25 + scale;

    end.x = v.x;
    end.y = v.y;
    end.z = 0.25;

    marker.points.emplace_back(start);
    marker.points.emplace_back(end);

    return marker;
}

visualization_msgs::Marker Sender::GetArrow(const Pose &p, double r, double g, double b, double a, double scale)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.id = id++;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;

    marker.scale.z = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;


    geometry_msgs::Point top;
    geometry_msgs::Point bottom_left;
    geometry_msgs::Point bottom_right;

    top.x = p.GetPosition().x;
    top.y = p.GetPosition().y;
    top.z = 0.0;

    Angle a1 = p.GetAngle();
    a1.Rotate(160);
    auto p1 = p.GetTranslation(1.5, a1);

    bottom_left.x = p1.x;
    bottom_left.y = p1.y;
    bottom_left.z = 0.0;

    Angle a2 = p.GetAngle();
    a2.Rotate(-160);
    auto p2 = p.GetTranslation(1.5, a2);
    bottom_right.x = p2.x;
    bottom_right.y = p2.y;
    bottom_right.z = 0.0;

    marker.points.emplace_back(top);
    marker.points.emplace_back(bottom_left);
    marker.points.emplace_back(bottom_right);
    return marker;
}

void Sender::Send()
{
    pub.publish(array);
    array.markers.clear();
}

void Sender::SendPoses(std::vector<Pose> poses, double r, double g, double b, double a, double z, double width)
{
    auto line_strip = GetLineStrip(std::move(poses), r, g, b, a, z, width);
    array.markers.emplace_back(line_strip);
    pub.publish(array);
}

std::vector<Pose> Sender::Translate(std::vector<Pose> poses, double length, double theta)
{
    std::vector<Pose> res;
    for(auto & p : poses)
    {
        Angle a = p.GetAngle();
        a.Rotate(theta);
        p.Translate(length, a);
        res.emplace_back(p);
    }
    return res;
}

void Sender::AddStartPoint(const Coor &v)
{
    visualization_msgs::Marker m = GetCone(v, 1.0, 1.0, 1.0, 1.0, 3.0);
    array.markers.emplace_back(m);
}

void Sender::AddEndPoint(const Coor &v)
{
    visualization_msgs::Marker m = GetCone(v, 0, 255.0 / 255, 128.0 / 255, 1.0, 3.0);
    array.markers.emplace_back(m);
}


void Sender::Clear()
{
    array.markers.clear();
}

void Sender::AddAnchor(const Pose &p, int id)
{
    visualization_msgs::Marker m = GetCone(p.GetPosition(), 0, 1.0, 0, 1.0, 5.0);
    array.markers.emplace_back(m);
    std::string context = "Anchor[" + std::to_string(id) + "] : ( " + std::to_string(p.x) + " " + std::to_string(p.y) + " " + std::to_string(p.GetAngle().Value()) + " )";
    visualization_msgs::Marker t = GetText(context, p.GetPosition(), 6);
    array.markers.emplace_back(t);
}

