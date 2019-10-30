#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>


struct DataFrame { // represents the available sensor information at the same time instance

    cv::Mat cameraImg; // camera image

    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
};

struct PerformanceStatistic {

    int keypointsTotal[10];
    int keypointsROI[10];
    int keypointsMatched[10]; // for project, use BF matching and descriptor distance ratio 0.8

    std::string detectorType;
    std::string descriptorType;

    double detectionTime[10];
    double descriptionTime[10];
    double combinedTime[10];

};

#endif /* dataStructures_h */
