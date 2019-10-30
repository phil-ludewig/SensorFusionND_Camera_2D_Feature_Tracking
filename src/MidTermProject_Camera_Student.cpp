/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/*
- Loads images into ring buffer
- Keypoint extraction
- Keypoint description
- Descriptor matching
- Performance evaluation
*/

// csv file for performance report
string filename = "PerformanceReport.csv";
ofstream outputFile(filename, ios::out|ios::trunc);

int main(int argc, const char *argv[])
{

    // #### INIT VARIABLES AND DATA STRUCTURES ####

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    // ### create all detector/descriptor combinations and initialize performance struct
    vector<string> detectorTypes = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
    vector<string> descriptorTypes = {"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};
    vector<PerformanceStatistic> typeCombinations; // performance struct for each detector/descriptor combination (see dataStructures.h)

    for(auto detector = detectorTypes.begin(); detector != detectorTypes.end(); ++detector)
    {
        for(auto descriptor = descriptorTypes.begin(); descriptor != descriptorTypes.end(); ++descriptor)
        {
            PerformanceStatistic newCombination;
            newCombination.detectorType = (*detector);
            newCombination.descriptorType = (*descriptor);
            for(int i=0; i < 10; i++) // modify if performance test on more than 10 images
            {
                newCombination.keypointsTotal[i] = 0;
                newCombination.keypointsROI[i] = 0;
                newCombination.keypointsMatched[i] = 0;
                newCombination.detectionTime[i] = 0.0f;
                newCombination.descriptionTime[i] = 0.0f;
            }
            typeCombinations.push_back(newCombination);
        }
    }

    // #### Loop over all det/desc combinations ####
    for(auto currCombo = typeCombinations.begin(); currCombo != typeCombinations.end(); ++currCombo)
    {
        // non working detector/descriptor combinations
        if(((*currCombo).detectorType.compare("SHITOMASI") == 0 && (*currCombo).descriptorType.compare("BRIEF") == 0)
            || ((*currCombo).detectorType.compare("SHITOMASI") == 0 && (*currCombo).descriptorType.compare("ORB") == 0)
            || ((*currCombo).detectorType.compare("SHITOMASI") == 0 && (*currCombo).descriptorType.compare("SIFT") == 0)
            || ((*currCombo).detectorType.compare("SHITOMASI") == 0 && (*currCombo).descriptorType.compare("AKAZE") == 0)
            || ((*currCombo).detectorType.compare("HARRIS") == 0 && (*currCombo).descriptorType.compare("BRIEF") == 0)
            || ((*currCombo).detectorType.compare("HARRIS") == 0 && (*currCombo).descriptorType.compare("ORB") == 0)
            || ((*currCombo).detectorType.compare("HARRIS") == 0 && (*currCombo).descriptorType.compare("AKAZE") == 0)
            || ((*currCombo).detectorType.compare("HARRIS") == 0 && (*currCombo).descriptorType.compare("SIFT") == 0)
            || ((*currCombo).detectorType.compare("FAST") == 0 && (*currCombo).descriptorType.compare("BRIEF") == 0)
            || ((*currCombo).detectorType.compare("FAST") == 0 && (*currCombo).descriptorType.compare("ORB") == 0)
            || ((*currCombo).detectorType.compare("FAST") == 0 && (*currCombo).descriptorType.compare("AKAZE") == 0)
            || ((*currCombo).detectorType.compare("FAST") == 0 && (*currCombo).descriptorType.compare("SIFT") == 0)
            || ((*currCombo).detectorType.compare("BRISK") == 0 && (*currCombo).descriptorType.compare("BRIEF") == 0)
            || ((*currCombo).detectorType.compare("BRISK") == 0 && (*currCombo).descriptorType.compare("ORB") == 0)
            || ((*currCombo).detectorType.compare("BRISK") == 0 && (*currCombo).descriptorType.compare("AKAZE") == 0)
            || ((*currCombo).detectorType.compare("BRISK") == 0 && (*currCombo).descriptorType.compare("SIFT") == 0)
            || ((*currCombo).detectorType.compare("ORB") == 0 && (*currCombo).descriptorType.compare("BRIEF") == 0)
            || ((*currCombo).detectorType.compare("ORB") == 0 && (*currCombo).descriptorType.compare("ORB") == 0)
            || ((*currCombo).detectorType.compare("ORB") == 0 && (*currCombo).descriptorType.compare("AKAZE") == 0)
            || ((*currCombo).detectorType.compare("ORB") == 0 && (*currCombo).descriptorType.compare("SIFT") == 0)
            || ((*currCombo).detectorType.compare("AKAZE") == 0 && (*currCombo).descriptorType.compare("BRIEF") == 0)
            || ((*currCombo).detectorType.compare("AKAZE") == 0 && (*currCombo).descriptorType.compare("ORB") == 0)
            || ((*currCombo).detectorType.compare("AKAZE") == 0 && (*currCombo).descriptorType.compare("SIFT") == 0)
            || ((*currCombo).detectorType.compare("AKAZE") == 0 && (*currCombo).descriptorType.compare("AKAZE") == 0)
            || ((*currCombo).detectorType.compare("SIFT") == 0 && (*currCombo).descriptorType.compare("BRIEF") == 0)
            || ((*currCombo).detectorType.compare("SIFT") == 0 && (*currCombo).descriptorType.compare("ORB") == 0)
            || ((*currCombo).detectorType.compare("SIFT") == 0 && (*currCombo).descriptorType.compare("AKAZE") == 0)
        )
        {
            continue; // skip non compatible (errors at matching) det/desc combination
        }
        cout << "Current combo: " << (*currCombo).detectorType << " " << (*currCombo).descriptorType << endl;
        // #### Loop over all images ####
        //for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
        for (size_t imgIndex = 0; imgIndex < 10; imgIndex++)
        {
            // #### Load images into ring buffer ####
            // assemble filenames for current index
            ostringstream imgNumber;
            imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
            string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

            // convert image to grayscale
            cv::Mat img, imgGray;
            img = cv::imread(imgFullFilename);
            cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

            // push image into data frame buffer
            DataFrame frame;
            frame.cameraImg = imgGray;

            if(dataBuffer.size() < dataBufferSize)
                dataBuffer.push_back(frame);

            else
            {
                dataBuffer.erase(dataBuffer.begin());
                dataBuffer.push_back(frame);
            }

            //cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

            // #### DETECT IMAGE KEYPOINTS ####

            vector<cv::KeyPoint> keypoints; // keypoints for current image
            string detectorType = (*currCombo).detectorType; // detectors: SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
            bool visKeypoints = false;

            if (detectorType.compare("SHITOMASI") == 0)
                (*currCombo).detectionTime[imgIndex] = detKeypointsShiTomasi(keypoints, imgGray, visKeypoints);

            else if(detectorType.compare("HARRIS") == 0)
                (*currCombo).detectionTime[imgIndex] = detKeypointsHarris(keypoints, imgGray, visKeypoints);

            else
                (*currCombo).detectionTime[imgIndex] = detKeypointsModern(keypoints, imgGray, detectorType, visKeypoints);

            (*currCombo).keypointsTotal[imgIndex] = keypoints.size();

            // ### optional: only keep keypoints on the preceding vehicle (simulate bounding box)
            bool bFocusOnVehicle = true;
            cv::Rect vehicleRect(535, 180, 180, 150);
            vector<cv::KeyPoint> croppedKeypoints;

            if (bFocusOnVehicle)
            {
                for(auto it = keypoints.begin(); it != keypoints.end(); ++it) // cycle through keypoints
                {
                    if(vehicleRect.contains((*it).pt))
                        croppedKeypoints.push_back(*it);
                }
                //cout << "Bounding box focusing removed " << keypoints.size() - croppedKeypoints.size() << " outliers." << endl;
                (*currCombo).keypointsROI[imgIndex] = croppedKeypoints.size();
                keypoints = croppedKeypoints; // replace old list
            }

            // ### optional : limit number of keypoints for debugging
            bool bLimitKpts = false;
            if (bLimitKpts)
            {
                int maxKeypoints = 30;

                if (detectorType.compare("SHITOMASI") == 0)
                { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                    keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                }
                cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                //cout << " NOTE: Keypoints have been limited!" << endl;
            }

            (dataBuffer.end() - 1)->keypoints = keypoints; // push keypoints and descriptor for current frame to end of data buffer

            //cout << "#2 : DETECT KEYPOINTS done" << endl;


            // #### EXTRACT KEYPOINT DESCRIPTORS ####

            cv::Mat descriptors;
            string descriptorType = (*currCombo).descriptorType; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT

            (*currCombo).descriptionTime[imgIndex] = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);

            (*currCombo).combinedTime[imgIndex] = (*currCombo).descriptionTime[imgIndex] + (*currCombo).detectionTime[imgIndex];
            (dataBuffer.end() - 1)->descriptors = descriptors; // push descriptors for current frame to end of data buffer

            //cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

            if (dataBuffer.size() > 1) // only attempt matching if at least two images have been processed
            {

                // #### MATCH KEYPOINT DESCRIPTORS ####

                vector<cv::DMatch> matches;
                string matcherType = "MAT_BF";        // MAT_BF (brute force), MAT_FLANN
                string descriptorType = "DES_BINARY"; // DES_BINARY (Binary based: BRIEF, BRISK, ORB, FREAK, KAZE)
                if((*currCombo).descriptorType.compare("SIFT") == 0) // SIFT only works with MAT_FLANN and DES_HOG
                {
                    descriptorType = "DES_HOG"; // DES_HOG (Gradient based, slower type: SIFT)
                    matcherType = "MAT_FLANN";
                }

                string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

                matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                 (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                 matches, descriptorType, matcherType, selectorType);

                (*currCombo).keypointsMatched[imgIndex] = matches.size();
                (dataBuffer.end() - 1)->kptMatches = matches; // store matches in current data frame

                //cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

                // visualize matches between current and previous image
                //bVis = true;
                if (bVis)
                {
                    cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                    cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                    (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                    matches, matchImg,
                                    cv::Scalar::all(-1), cv::Scalar::all(-1),
                                    vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                    string windowName = "Matching keypoints between two camera images";
                    cv::namedWindow(windowName, 7);
                    cv::imshow(windowName, matchImg);
                    //cout << "Press key to continue to next image" << endl;
                    cv::waitKey(0); // wait for key to be pressed
                }
                bVis = false;
            }

        } // end of images loop
    } // end of combo loop

    // #### Performance Report
    outputFile  << "Detector Type" << ","
                << "DescriptorType" << ","
                << "Frame" << ","
                << "Total Keypoints" << ","
                << "Keypoints in ROI" << ","
                << "Keypoints in ROI matched" << ","
                << "Time for detection [ms]" << ","
                << "Time for description [ms]" << ","
                << "Combined Time [ms]" << ","
                << endl;

    for(auto it = typeCombinations.begin(); it != typeCombinations.end(); ++it)
    {
        for(int i = 0; i < 10; i++)
        {
            if((*it).keypointsTotal[i] == 0) // combination was not compatible
            {
                outputFile  << (*it).detectorType << ","
                            << (*it).descriptorType << ","
                            << "N/A" << ","
                            << "N/A" << ","
                            << "N/A" << ","
                            << "N/A" << ","
                            << "N/A" << ","
                            << "N/A" << ","
                            << "N/A" << ","
                            << endl;
            }

            else
            {
                outputFile  << (*it).detectorType << ","
                            << (*it).descriptorType << ","
                            << i << ","
                            << (*it).keypointsTotal[i] << ","
                            << (*it).keypointsROI[i] << ","
                            << (*it).keypointsMatched[i] << ","
                            << (*it).detectionTime[i] << ","
                            << (*it).descriptionTime[i] << ","
                            << (*it).combinedTime[i] << ","
                            << endl;
            }
        }
    }
    outputFile.close();

    return 0;

}
