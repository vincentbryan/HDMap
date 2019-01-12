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

void Road::OnSend(Sender &sender)
{
    char _buf[128];
    sprintf(_buf,"Road[%d]: %4.2fm, %4.2fdeg",
            ID, Lenght, GetReferenceLinePoses().front().GetAngle().Value());
    std::string text = _buf;

    auto p1 = GetReferenceLinePoses().front();
    p1.Rotate(-90);
    p1.Translate(4.0, p1.GetAngle());

    auto p2 = GetReferenceLinePoses().back();
    p2.Rotate(-90);
    p2.Translate(4.0, p2.GetAngle());

    auto m1 = sender.GetText(text, p1.GetPosition());
    auto m2 = sender.GetText(text, p2.GetPosition());

    sender.array.markers.emplace_back(m1);
    sender.array.markers.emplace_back(m2);

    //sender.Send();

    for(auto & s : mSecPtrs) s->OnSend(sender);
    for(auto & s : mSigPtrs) s->OnSend(sender);
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

std::vector<SigPtr> Road::GetSignals()
{
    return mSigPtrs;
}

void Road::GenerateRegionPoses() {
    if (mRegionPoses.empty())
    {
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

std::vector<Pose> Road::GetReferenceLinePoses()
{
    std::vector<Pose> poses;
    for(const auto& sp: mSecPtrs){
        const auto& sec_poses = sp->GetReferenceLinePose();
        poses.insert(poses.end(),sec_poses.begin(),sec_poses.end());
    }
    return poses;
}

std::vector<Pose> Road::GetRightmostLinePoses()
{
    std::vector<Pose> poses;
    for(const auto &sp: mSecPtrs){
        const auto& sp_pose = sp->GetLanePoseByIndex(sp->mRightBoundary);
        poses.insert(poses.end(),sp_pose.begin(),sp_pose.end());
    }
    return poses;
}



