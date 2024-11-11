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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>

#define PEOPLE_LABLE         1
#define BICYCLE_LABEL        2
#define CAR_LABEL            3
#define MOTORBIKE_LABEL      4
#define BUS_LABEL            5
#define train                6
#define TRUCK_LABEL          7 // いったんこれ
#define boat                 8
#define traffic_light        9
#define bicycler             10
#define braille_block        11
#define guardrail            12
#define white_line           13
#define CROSSWALK_LABEL      14
#define signal_button        15
#define SIGNAL_RED_LABEL     16
#define signal_blue          17
#define stairs               18
#define handrail             19
#define steps                20
#define faregates            21
#define train_ticket_machine 22
#define shrubs               23
#define tree                 24
#define vending_machine      25
#define bathroom             26
#define door                 27
#define elevator             28
#define escalator            29
#define bollard              30
#define bus_stop_sign        31
#define pole                 32
#define MONUMENT_LABEL       33
#define FENCE_LABEL          34
#define wall                 35
#define signboard            36
#define FLAG_LABEL           37
#define postbox              38
#define safetycone           39
#define SIDEWALK_LABEL       40
#define ROAD_LABEL           41

namespace ORB_SLAM3
{

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST,
                 int cameraWidth,int cameraHeight);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    int operator()( cv::InputArray _image, cv::InputArray _segment, cv::InputArray _mask,
                    std::vector<cv::KeyPoint>& _keypoints, cv::OutputArray _descriptors,
                    std::vector<int> &vLappingArea, std::vector< std::vector<cv::Point> >& _convexHulls);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;
    std::vector<cv::Mat> mvImagePyramidSegment;
    void CheckWithConvexHull(std::vector<std::vector<cv::KeyPoint>>& mvKeysT,std::vector<cv::Point2f> T, std::vector<std::vector<cv::Point>>& convexHulls);
    void ProcessMovingObject(const cv::Mat &imgray );
    void ComputeConvexhullFromMask(std::vector< std::vector<cv::Point> >& convexhull, int targetLabel);

    // The previous image
    cv::Mat imGrayPre;
    std::vector<cv::Point2f> prepoint, nextpoint;
    std::vector<cv::Point2f> F_prepoint, F_nextpoint;
    std::vector<cv::Point2f> F2_prepoint, F2_nextpoint;

    std::vector<uchar> state;
    std::vector<float> err;
    std::vector<std::vector<cv::KeyPoint>> mvKeysPre;
    std::vector<cv::Point2f> T_M;
    double limit_dis_epi =1; 
    double limit_of_check = 2120;
    int limit_edge_corner = 5;

    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    double orbExtractTime;
    double movingDetectTime;

protected:

    void ComputePyramid(cv::Mat image);
    void ComputePyramidSegment(cv::Mat imS);
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::Point> pattern;

    // int mostFrequentElement(const std::vector<int>& class_indices);

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;
    int cameraWidth,cameraHeight;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM

#endif

