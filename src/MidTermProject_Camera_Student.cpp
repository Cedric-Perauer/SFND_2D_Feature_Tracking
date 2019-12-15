/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <deque>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)
    std::string filename = "/home/cedric/SFND_2D_Tracking/data/data.csv";
    std::ofstream output_stream(filename, std::ios::binary);


    if (!output_stream.is_open()) {
        std::cerr << "failed to open file: " << filename << std::endl;
        return EXIT_FAILURE;
    }

    // write CSV header row
    output_stream << "Detector Type" << ","
                  << "Descriptor Type" << ","
                  << "Frame#" << ","
                  << "#KeyPointsPerFrame" << ","
                  << "#KeyPointsPerROI" << ","
                  << "DetectorTime(ms)" << ","
                  << "DescriptorTime(ms)" << ","
                  << "#MatchedPoints" << "," << "MatchingTime(ms))" << std::endl;
    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    deque<DataFrame> ring_buffer;
    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */
    vector<string> detectors = {"SHITOMASI","HARRIS","FAST","BRISK","ORB","AKAZE","SIFT"};
    vector<string> descriptors = {"BRISK","BRIEF","ORB","FREAK","AKAZE","SIFT"};
    for(auto detectorType : detectors)
    {
        for (auto descriptorType : descriptors)
        {

            if( !((detectorType=="SHITOMASI")&&(descriptorType=="AKAZE")) && !((detectorType=="HARRIS")&&(descriptorType=="AKAZE")) && !((detectorType=="FAST")&&(descriptorType=="AKAZE"))
            && !((detectorType=="BRISK")&&(descriptorType=="AKAZE")) && !((detectorType=="SIFT")&&(descriptorType=="ORB"))  && !((detectorType=="SIFT")&&(descriptorType=="AKAZE"))  && !((detectorType=="ORB")&&(descriptorType=="AKAZE")))
            {
                for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++) {
                    /* LOAD IMAGE INTO BUFFER */

                    // assemble filenames for current index
                    ostringstream imgNumber;
                    imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
                    string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

                    // load image from file and convert to grayscale
                    cv::Mat img, imgGray;
                    img = cv::imread(imgFullFilename);
                    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

                    // push image into data frame buffer
                    DataFrame frame;
                    frame.cameraImg = imgGray;
                    ring_buffer.push_back(frame);

                    if (ring_buffer.size() > dataBufferSize) { ring_buffer.pop_front(); }
                    //// EOF STUDENT ASSIGNMENT
                    cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

                    /* DETECT IMAGE KEYPOINTS */

                    // extract 2D keypoints from current image
                    vector<cv::KeyPoint> keypoints; // create empty feature list for current image
                    int keypoints_all_num = 0;
                    int keypoints_roi_num = 0;
                    int num_matches = 0;

                    double detector_t = (double)cv::getTickCount();
                    if (detectorType.compare("SHITOMASI") == 0) {
                        detKeypointsShiTomasi(keypoints, imgGray, false);
                    } else if (detectorType.compare("HARRIS") == 0) {
                        detKeypointsHarris(keypoints, imgGray, false);

                    } else {
                        detKeypointsModern(keypoints, imgGray, detectorType, bVis);
                    }

                    detector_t = ((double)cv::getTickCount() - detector_t) / cv::getTickFrequency();
                    keypoints_all_num = keypoints.size();
                    // only keep keypoints on the preceding vehicle
                    bool bFocusOnVehicle = true;
                    cv::Rect vehicleRect(535, 180, 180, 150);
                    if (bFocusOnVehicle) {
                        vector<cv::KeyPoint> kpts_roi;
                        for(auto kpt : keypoints)
                        {
                            if (vehicleRect.contains(kpt.pt))
                                kpts_roi.push_back(kpt);
                        }
                        keypoints = kpts_roi;
                    }
                    keypoints_roi_num = keypoints.size();
                    cout << "Keypoints in ROI " << keypoints_roi_num << endl;

                    // optional : limit number of keypoints (helpful for debugging and learning)
                    bool bLimitKpts = false;
                    if (bLimitKpts) {
                        int maxKeypoints = 50;

                        if (detectorType.compare("SHITOMASI") ==
                            0) { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                            keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                        }
                        cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                        cout << " NOTE: Keypoints have been limited!" << endl;
                    }

                    // push keypoints and descriptor for current frame to end of data buffer
                    (ring_buffer.end() - 1)->keypoints = keypoints;

                    cv::Mat descriptors;
                    double descriptor_t = (double)cv::getTickCount();
                    descKeypoints((ring_buffer.end() - 1)->keypoints, (ring_buffer.end() - 1)->cameraImg, descriptors,
                                  descriptorType);
                    descriptor_t = ((double)cv::getTickCount() - descriptor_t) / cv::getTickFrequency();
                    // push descriptors for current frame to end of data buffer
                    (ring_buffer.end() - 1)->descriptors = descriptors;

                    double match_t;
                    if (ring_buffer.size() > 1) // wait until at least two images have been processed
                    {


                        vector<cv::DMatch> matches;
                        string matcherType = "MAT_BF"; // MAT_BF, MAT_FLANN
                        string desc = "DES_BINARY"; // DES_BINARY, DES_HOG
                        if(descriptorType == "SIFT")
                            desc = "DES_HOG";
                        string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

                        //// STUDENT ASSIGNMENT
                        //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                        //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
                        match_t = (double)cv::getTickCount();
                        matchDescriptors((ring_buffer.end() - 2)->keypoints, (ring_buffer.end() - 1)->keypoints,
                                         (ring_buffer.end() - 2)->descriptors, (ring_buffer.end() - 1)->descriptors,
                                         matches, desc, matcherType, selectorType);
                        match_t = ((double)cv::getTickCount() - match_t) / cv::getTickFrequency();
                        //// EOF STUDENT ASSIGNMENT

                        // store matches in current data frame
                        (ring_buffer.end() - 1)->kptMatches = matches;

                        num_matches = matches.size();
                        // visualize matches between current and previous image
                        bVis = false;
                        if (bVis) {
                            cv::Mat matchImg = ((ring_buffer.end() - 1)->cameraImg).clone();
                            cv::drawMatches((ring_buffer.end() - 2)->cameraImg, (ring_buffer.end() - 2)->keypoints,
                                            (ring_buffer.end() - 1)->cameraImg, (ring_buffer.end() - 1)->keypoints,
                                            matches, matchImg,
                                            cv::Scalar::all(-1), cv::Scalar::all(-1),
                                            vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                            string windowName = "Matching keypoints between two camera images";
                            cv::namedWindow(windowName, 7);
                            cv::imshow(windowName, matchImg);
                            cout << "Press key to continue to next image" << endl;
                            cv::waitKey(0); // wait for key to be pressed
                        }
                        bVis = false;
                    }

                   /* if(num_matches < 0)
                        num_matches = 0;
                    detector_t = (1000 * detector_t/1.0);
                    descriptor_t = (1000 * descriptor_t/1.0);
                    match_t = (1000 * match_t /1.0);
                    output_stream << detectorType
                                  << "," << descriptorType
                                  << "," << imgIndex
                                  << "," << keypoints_all_num
                                  << "," << keypoints_roi_num
                                  << "," << std::fixed << std::setprecision(8) << detector_t
                                  << "," << std::fixed << std::setprecision(8) << descriptor_t
                                  << "," << num_matches
                                  << "," << std::fixed << std::setprecision(8) << match_t << std::endl;
*/
                } // eof loop over all images
                output_stream << std::endl;
            }
        }
    }
    output_stream.close();

    return 0;
}