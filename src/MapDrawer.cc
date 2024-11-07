/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <mutex>

namespace ORB_SLAM3
{

    // std::vector<cv::Scalar> color_map = {
    //     cv::Scalar(0, 0, 0),
    //     cv::Scalar(255, 0, 0),    // Red
    //     cv::Scalar(0, 255, 0),    // Green
    //     cv::Scalar(0, 0, 255),    // Blue
    //     cv::Scalar(255, 255, 0),  // Yellow
    //     cv::Scalar(0, 255, 255),  // Cyan
    //     cv::Scalar(255, 0, 255),  // Magenta
    //     cv::Scalar(192, 192, 192),// Silver
    //     cv::Scalar(128, 128, 128),// Gray
    //     cv::Scalar(128, 0, 0),    // Maroon
    //     cv::Scalar(128, 128, 0),  // Olive
    //     cv::Scalar(0, 128, 0),    // Dark Green
    //     cv::Scalar(128, 0, 128),  // Purple
    //     cv::Scalar(0, 128, 128),  // Teal
    //     cv::Scalar(0, 0, 128),    // Navy
    //     cv::Scalar(255, 165, 0),  // Orange
    //     cv::Scalar(255, 215, 0),  // Gold
    //     cv::Scalar(0, 100, 0),    // Dark Green
    //     cv::Scalar(75, 0, 130),   // Indigo
    //     cv::Scalar(238, 130, 238),// Violet
    //     cv::Scalar(240, 248, 255),// Alice Blue
    //     cv::Scalar(245, 245, 220),// Beige
    //     cv::Scalar(255, 228, 196),// Bisque
    //     cv::Scalar(0, 255, 0),      // Green
    //     cv::Scalar(255, 235, 205),// Blanched Almond
    //     cv::Scalar(138, 43, 226), // Blue Violet
    //     cv::Scalar(222, 184, 135),// Burly Wood
    //     cv::Scalar(95, 158, 160), // Cadet Blue
    //     cv::Scalar(127, 255, 0),  // Chartreuse
    //     cv::Scalar(210, 105, 30), // Chocolate
    //     cv::Scalar(255, 127, 80), // Coral
    //     cv::Scalar(100, 149, 237),// Cornflower Blue
    //     cv::Scalar(255, 248, 220),// Cornsilk
    //     cv::Scalar(220, 20, 60),  // Crimson
    //     cv::Scalar(0, 255, 127),  // Spring Green
    //     cv::Scalar(70, 130, 180), // Steel Blue
    //     cv::Scalar(244, 164, 96), // Sandy Brown
    //     cv::Scalar(176, 224, 230),// Powder Blue
    //     cv::Scalar(188, 143, 143),// Rosy Brown
    //     cv::Scalar(255, 160, 122),// Light Salmon
    //     cv::Scalar(128, 200, 200),// Custom Color
    //     cv::Scalar(0, 255, 0)       // Green
    // };

    std::vector<cv::Scalar> color_map = {
    // cv::Scalar(Blue, Green, Red)
    // cv::Scalar(0, 0, 0),        // background
    cv::Scalar(0, 0, 0),        // 0: person
    cv::Scalar(0, 255, 0),      // 1: bicycle
    cv::Scalar(255, 0, 0),      // 2: car
    cv::Scalar(0, 255, 255),    // 3: motorbike
    cv::Scalar(255, 255, 0),    // 4: bus
    cv::Scalar(255, 0, 255),    // 5: train
    cv::Scalar(0, 255, 255),    // 6: truck
    cv::Scalar(0, 0, 255),      // 7: boat
    cv::Scalar(0, 255, 0),      // 8: traffic_light
    cv::Scalar(255, 0, 0),      // 9: bicycler
    cv::Scalar(0, 128, 0),      // braille_block
    cv::Scalar(128, 0, 128),    // guardrail
    cv::Scalar(128, 128, 0),    // white_line
    cv::Scalar(0, 0, 255),      // 13: crosswalk
    cv::Scalar(0, 165, 255),    // 14: signal_button
    cv::Scalar(0, 0, 255),    // 15: signal_red
    cv::Scalar(0, 100, 0),      // 16: signal_blue
    cv::Scalar(130, 0, 75),     // stairs
    cv::Scalar(238, 130, 238),  // handrail
    cv::Scalar(255, 248, 240),  // steps
    cv::Scalar(220, 245, 245),  // faregates
    cv::Scalar(196, 228, 255),  // train_ticket_machine
    cv::Scalar(0, 255, 0),      // shrubs
    cv::Scalar(205, 235, 255),  // tree
    cv::Scalar(226, 43, 138),   // vending_machine
    cv::Scalar(135, 184, 222),  // bathroom
    cv::Scalar(160, 158, 95),   // door
    cv::Scalar(0, 255, 127),    // elevator
    cv::Scalar(30, 105, 210),   // escalator
    cv::Scalar(80, 127, 255),   // bollard
    cv::Scalar(237, 149, 100),  // bus_stop_sign
    cv::Scalar(220, 248, 255),  // pole
    cv::Scalar(60, 20, 220),    // monument
    cv::Scalar(127, 255, 0),    // fence
    cv::Scalar(180, 130, 70),   // wall
    cv::Scalar(96, 164, 244),   // signboard
    cv::Scalar(230, 224, 176),  // flag
    cv::Scalar(143, 143, 188),  // postbox
    cv::Scalar(128, 200, 200),  // safety-cone
    cv::Scalar(122, 160, 255),  // sidewalk
    cv::Scalar(255, 0, 255)       // road
};


MapDrawer::MapDrawer(Atlas* pAtlas, const string &strSettingPath, Settings* settings):mpAtlas(pAtlas)
{
    if(settings){
        newParameterLoader(settings);
    }
    else{
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        bool is_correct = ParseViewerParamFile(fSettings);

        if(!is_correct)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }
}

void MapDrawer::newParameterLoader(Settings *settings) {
    mKeyFrameSize = settings->keyFrameSize();
    mKeyFrameLineWidth = settings->keyFrameLineWidth();
    mGraphLineWidth = settings->graphLineWidth();
    mPointSize = settings->pointSize();
    mCameraSize = settings->cameraSize();
    mCameraLineWidth  = settings->cameraLineWidth();
}

bool MapDrawer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::FileNode node = fSettings["Viewer.KeyFrameSize"];
    if(!node.empty())
    {
        mKeyFrameSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.KeyFrameLineWidth"];
    if(!node.empty())
    {
        mKeyFrameLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.GraphLineWidth"];
    if(!node.empty())
    {
        mGraphLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.GraphLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.PointSize"];
    if(!node.empty())
    {
        mPointSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.PointSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraSize"];
    if(!node.empty())
    {
        mCameraSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraLineWidth"];
    if(!node.empty())
    {
        mCameraLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

void MapDrawer::setGlColor(const cv::Scalar& color) {
    float r = color[2] / 255.0f;
    float g = color[1] / 255.0f;
    float b = color[0] / 255.0f;

    // 色を設定
    glColor3f(r, g, b);
}

void MapDrawer::DrawMapPoints()
{
    Map* pActiveMap = mpAtlas->GetCurrentMap();
    if(!pActiveMap)
        return;

    const vector<MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        
        // cv::Scalar color = (vpMPs[i]->mClassIdx != 40) ? color_map[vpMPs[i]->mClassIdx] : cv::Scalar(0,0,0);
        // float r = color[2] / 255.0f;
        // float g = color[1] / 255.0f;
        // float b = color[0] / 255.0f;
        // glColor3f(r, g, b);
        // if (vpMPs[i]->mClassIdx!=0) std::cout << vpMPs[i]->mClassIdx << std::endl;
        // if (vpMPs[i]->mClassIdx != 0) std::cout << "class idx " << vpMPs[i]->mClassIdx << ", ";
        Eigen::Matrix<float,3,1> pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    // glColor3f(0.0,1.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        // setGlColor(color_map[vpMPs[i]->mClassIdx]);
        Eigen::Matrix<float,3,1> pos = (*sit)->GetWorldPos();
        cv::Scalar color = (*sit)->mClassIdx!=40 ? color_map[(*sit)->mClassIdx] : cv::Scalar(0,0,0);
        float r = color[2] / 255.0f;
        float g = color[1] / 255.0f;
        float b = color[0] / 255.0f;
        // if ((*sit)->mClassIdx != 0) std::cout << "class idx " << (*sit)->mClassIdx << ", ";
        glColor3f(r, g, b);
        glVertex3f(pos(0),pos(1),pos(2));

    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph, const bool bDrawOptLba)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    Map* pActiveMap = mpAtlas->GetCurrentMap();
    // DEBUG LBA
    std::set<long unsigned int> sOptKFs = pActiveMap->msOptKFs;
    std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;

    if(!pActiveMap)
        return;

    const vector<KeyFrame*> vpKFs = pActiveMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
            unsigned int index_color = pKF->mnOriginMapId;

            glPushMatrix();

            glMultMatrixf((GLfloat*)Twc.data());

            if(!pKF->GetParent()) // It is the first KF in the map
            {
                glLineWidth(mKeyFrameLineWidth*5);
                glColor3f(1.0f,0.0f,0.0f);
                glBegin(GL_LINES);
            }
            else
            {
                //cout << "Child KF: " << vpKFs[i]->mnId << endl;
                glLineWidth(mKeyFrameLineWidth);
                if (bDrawOptLba) {
                    if(sOptKFs.find(pKF->mnId) != sOptKFs.end())
                    {
                        glColor3f(0.0f,1.0f,0.0f); // Green -> Opt KFs
                    }
                    else if(sFixedKFs.find(pKF->mnId) != sFixedKFs.end())
                    {
                        glColor3f(1.0f,0.0f,0.0f); // Red -> Fixed KFs
                    }
                    else
                    {
                        glColor3f(0.0f,0.0f,1.0f); // Basic color
                    }
                }
                else
                {
                    glColor3f(0.0f,0.0f,1.0f); // Basic color
                }
                glBegin(GL_LINES);
            }

            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();

            glEnd();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        // cout << "-----------------Draw graph-----------------" << endl;
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            Eigen::Vector3f Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    Eigen::Vector3f Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow(0),Ow(1),Ow(2));
                    glVertex3f(Ow2(0),Ow2(1),Ow2(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                Eigen::Vector3f Owp = pParent->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owp(0),Owp(1),Owp(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                Eigen::Vector3f Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owl(0),Owl(1),Owl(2));
            }
        }

        glEnd();
    }

    if(bDrawInertialGraph && pActiveMap->isImuInitialized())
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(1.0f,0.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        //Draw inertial links
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKFi = vpKFs[i];
            Eigen::Vector3f Ow = pKFi->GetCameraCenter();
            KeyFrame* pNext = pKFi->mNextKF;
            if(pNext)
            {
                Eigen::Vector3f Owp = pNext->GetCameraCenter();
                glVertex3f(Ow(0),Ow(1),Ow(2));
                glVertex3f(Owp(0),Owp(1),Owp(2));
            }
        }

        glEnd();
    }

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();

    if(bDrawKF)
    {
        for(Map* pMap : vpMaps)
        {
            if(pMap == pActiveMap)
                continue;

            vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKF = vpKFs[i];
                Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
                unsigned int index_color = pKF->mnOriginMapId;

                glPushMatrix();

                glMultMatrixf((GLfloat*)Twc.data());

                if(!vpKFs[i]->GetParent()) // It is the first KF in the map
                {
                    glLineWidth(mKeyFrameLineWidth*5);
                    glColor3f(1.0f,0.0f,0.0f);
                    glBegin(GL_LINES);
                }
                else
                {
                    glLineWidth(mKeyFrameLineWidth);
                    glColor3f(mfFrameColors[index_color][0],mfFrameColors[index_color][1],mfFrameColors[index_color][2]);
                    glBegin(GL_LINES);
                }

                glVertex3f(0,0,0);
                glVertex3f(w,h,z);
                glVertex3f(0,0,0);
                glVertex3f(w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,h,z);

                glVertex3f(w,h,z);
                glVertex3f(w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(-w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);

                glVertex3f(-w,-h,z);
                glVertex3f(w,-h,z);
                glEnd();

                glPopMatrix();
            }
        }
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.inverse();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw)
{
    Eigen::Matrix4f Twc;
    {
        unique_lock<mutex> lock(mMutexCamera);
        Twc = mCameraPose.matrix();
    }

    for (int i = 0; i<4; i++) {
        M.m[4*i] = Twc(0,i);
        M.m[4*i+1] = Twc(1,i);
        M.m[4*i+2] = Twc(2,i);
        M.m[4*i+3] = Twc(3,i);
    }

    MOw.SetIdentity();
    MOw.m[12] = Twc(0,3);
    MOw.m[13] = Twc(1,3);
    MOw.m[14] = Twc(2,3);
}
} //namespace ORB_SLAM
