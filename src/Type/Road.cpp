//
// Created by vincent on 18-10-8.
//

#include <Type/Road.h>

using namespace hdmap;

unsigned int Road::ROAD_ID = 0;

Road::Road(Pose _start_pose)
{
    ID = ROAD_ID++;
    Lenght = 0;
    mPrevJid = mNextJid = -1;
    Direction = 0;
    mStartPose = _start_pose;
}

std::vector<Pose> Road::Trajectory(int begin_lane_idx, int end_lane_idx)
{
    if(mSecPtrs.size() == 1)
    {
        return mSecPtrs.back()->GetLanePoseByIndex(end_lane_idx);
    }

    std::vector<std::vector<int>> scheme;

    auto search = std::function<void(unsigned int, int, std::vector<int>)>();

    search = [&](unsigned int curr_sec_id, int curr_lane_idx, std::vector<int> v)
    {
        if(curr_sec_id + 1 == mSecPtrs.size()-1)
        {
            for(auto & m : mSecPtrs[curr_sec_id]->mLanes[curr_lane_idx].mSuccessors)
            {
                if(m == end_lane_idx)
                {
                    v.emplace_back(curr_lane_idx);
                    v.emplace_back(end_lane_idx);
                    scheme.emplace_back(v);
                }
            }
        }
        else
        {
            for(auto & x : mSecPtrs[curr_sec_id]->mLanes[curr_lane_idx].mSuccessors)
            {
                v.emplace_back(curr_lane_idx);
                search(curr_sec_id+1, x, v);
                v.pop_back();
            }
        }
    };

    std::vector<int> v;
    search(0, begin_lane_idx, v);

    for(auto & x : scheme)
    {
        for(auto & y : x)
            std::cout << y << " ";
        std::cout << std::endl;
    }

    //TODO
    auto r = scheme.front();

    std::vector<Pose> res;
    for(int i = 0; i < mSecPtrs.size(); i++)
    {
        auto p = mSecPtrs[i]->GetLanePoseByIndex(r[i]);
        res.insert(res.end(), p.begin(), p.end());
    }
    return res;
}

void Road::Send(Sender &sender)
{
    std::string text = "Road[" + std::to_string(ID) + "]: " + std::to_string(Lenght);

    auto p1 = GetReferenceLinePoses().front();
    p1.Rotate(-90);
    p1.Translate(4.0, p1.GetAngle());

    auto p2 = GetReferenceLinePoses().back();
    p2.Rotate(-90);
    p2.Translate(4.0, p2.GetAngle());

    auto m1 = sender.GetText(text, p1.GetPosition());
    auto m2 = sender.GetText(text, p2.GetPosition());

    /* 
    GenerateRegionPoses();
    std::vector<Pose> roadRegionPs;
    for(auto & x : mRegionVertices) roadRegionPs.emplace_back(x, Angle(0));
    roadRegionPs.emplace_back(mRegionVertices.front(), Angle(0));

    auto m3 = sender.GetLineStrip(roadRegionPs, 76.0/255.0, 180.0/255.0, 231.0/255.0, 1.0, 0, 0.15);
    */
    
    sender.array.markers.emplace_back(m1);
    sender.array.markers.emplace_back(m2);
    // sender.array.markers.emplace_back(m3);
    sender.Send();

    for(auto & s : mSecPtrs) s->Send(sender);
    for(auto & s : mSigPtrs) s->Send(sender);
}

boost::property_tree::ptree Road::ToXML()
{
    pt::ptree p_road;

    p_road.add("<xmlattr>.id", ID);
    p_road.add("<xmlattr>.direction", Direction);
    p_road.add("<xmlattr>.length", Lenght);
    p_road.add("<xmlattr>.prev_jid", mPrevJid);
    p_road.add("<xmlattr>.next_jid", mNextJid);

    for(auto & s : mSecPtrs) p_road.add_child("laneSection", s->ToXML());
    for(auto & s : mSigPtrs) p_road.add_child("signal", s->ToXML());

    return p_road;
}


SecPtr Road::AddSection(const Pose &_end_pose, double _ctrl_len1, double _ctrl_len2)
{
    unsigned int sec_id_ = ID * 100 + mSecPtrs.size();
    double s_;
    Pose start_pose_ ;

    if(mSecPtrs.empty())
    {
        s_ = 0;
        start_pose_ = mStartPose;
    }
    else
    {
        s_ = mSecPtrs.back()->mStartS + mSecPtrs.back()->mReferLine.Length();
        start_pose_ = mSecPtrs.back()->mReferLine.GetEndPose();
    }

    auto refer_line_ = Bezier(start_pose_, _end_pose, _ctrl_len1, _ctrl_len2);
    SecPtr p(new LaneSection(sec_id_, s_, refer_line_));
    mSecPtrs.emplace_back(p);
    return p;
}

void Road::AddSignal(double _x, double _y, double _z, Angle dir, std::string _type, std::string _info)
{
    mSigPtrs.emplace_back(new Signal(_x, _y, _z, dir, _type, _info));
}

void Road::FromXML(const pt::ptree &p)
{
    ID = p.get<int>("<xmlattr>.id");
    Lenght = p.get<double>("<xmlattr>.length");
    mPrevJid = p.get<int>("<xmlattr>.prev_jid");
    mNextJid = p.get<int>("<xmlattr>.next_jid");

    for(auto s : p.get_child(""))
    {
        if(s.first == "laneSection")
        {
            SecPtr p_section(new LaneSection());
            p_section->FromXML(s.second);
            mSecPtrs.emplace_back(p_section);
        }

        if(s.first == "signal")
        {
            SigPtr p_signal(new Signal());
            p_signal->FromXML(s.second);
            mSigPtrs.emplace_back(p_signal);
        }
    }
}

bool Road::Cover(const Coor &v)
{
    for(auto & x : mSecPtrs)
    {
        if(x->Cover(v))
            return true;
    }
    return false;
}

std::vector<SigPtr> Road::GetSignals()
{
    return mSigPtrs;
}

void Road::GenerateRegionPoses() {
    if (mRegionPoses.empty()) {
        auto left_poses= GetReferenceLinePoses();
        auto right_poses = GetRightmostLinePoses();
        mRegionPoses.insert(mRegionPoses.end(), left_poses.rbegin(), left_poses.rend());
        mRegionPoses.insert(mRegionPoses.end(), right_poses.begin(), right_poses.end());

        // store kdtree catch for distance search
        mKdtreeData.clear();
        for (auto &p: mRegionPoses) {
            std::vector<double> _tmp{p.x, p.y};
            mKdtreeData.emplace_back(_tmp);
        }
        mKdtree.SetData(mKdtreeData, kt::VARIANCE);
    };
}

std::vector<Pose> Road::GetReferenceLinePoses() {
    std::vector<Pose> poses;
    for(const auto& sp: mSecPtrs){
        auto sec_poses = sp->GetReferPose();
        poses.insert(poses.end(),sec_poses.begin(),sec_poses.end());
    }
    return poses;
}

std::vector<Pose> Road::GetRightmostLinePoses() {
    std::vector<Pose> poses;
    for(const auto &sp: mSecPtrs){
        auto sp_pose = sp->GetLanePoseByIndex(sp->mRightBoundary);
        poses.insert(poses.end(),sp_pose.begin(),sp_pose.end());
    }
    return poses;
}

double Road::GetDistanceFromCoor(const Coor &v) {
    if (mRegionPoses.empty()) {
        GenerateRegionPoses();
    }

    if (Cover(v)) return 0;
    
    std::vector<int> indices;
    std::vector<double> distances;
    mKdtree.NearestSearch({v.x, v.y}, indices, distances, 1);
    return distances[0];
}

std::vector<Pose> Road::GetRegionPoses() {
    if (mRegionPoses.empty()) {
        GenerateRegionPoses();
    }
    return mRegionPoses;
}

