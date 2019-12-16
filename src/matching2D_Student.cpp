#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    // Brute force
    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType;

        // with SIFT
        if (descriptorType.compare("DES_HOG") == 0)
        {
            normType = cv::NORM_L2;
        }

            // with all other binary descriptors
        else if (descriptorType.compare("DES_BINARY") == 0)
        {
            normType = cv::NORM_HAMMING;
        }

        else {
            throw invalid_argument(descriptorType + " is not a valid descriptorCategory");
        }

        matcher = cv::BFMatcher::create(normType, crossCheck);
    }

        // FLANN matching
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        // with SIFT
        if (descriptorType.compare("DES_HOG") == 0)
        {
            matcher = cv::FlannBasedMatcher::create();
        }

            // with all other binary descriptorTypes
        else if (descriptorType.compare("DES_BINARY") == 0)
        {
            const cv::Ptr<cv::flann::IndexParams>& indexParams = cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2);
            matcher = cv::makePtr<cv::FlannBasedMatcher>(indexParams);
        }

        else {
            throw invalid_argument(descriptorType + " is not a valid descriptorCategory");
        }
    }

    else {
        throw invalid_argument(matcherType + " is not a valid matcherType");
    }

    // Perform nearest neighbor matching (best match)
    if (selectorType.compare("SEL_NN") == 0)
    {   if((kPtsRef.size()>=2 && kPtsSource.size()>=2) && (descSource.type()==descRef.type()) && (descSource.cols == descRef.cols))
        {matcher->match(descSource, descRef, matches);}
    }

        // Perform k nearest neighbors (k=2)
    else if (selectorType.compare("SEL_KNN") == 0)
    {
        int k = 2;
        vector<vector<cv::DMatch>> knn_matches;
        if(kPtsRef.size()>=2 && kPtsSource.size()>=2 && (descSource.type()==descRef.type()) && (descSource.cols == descRef.cols)  )
        {matcher->knnMatch(descSource, descRef, knn_matches, k);}

        // Filter matches using descriptor distance ratio test
        double minDescDistRatio = 0.8;
        for (auto it : knn_matches) {
            // The returned knn_matches vector contains some nested vectors with size < 2 !?
            if ( 2 == it.size() && (it[0].distance < minDescDistRatio * it[1].distance)) {
                matches.push_back(it[0]);
            }
        }
    }

    else {
        throw invalid_argument(selectorType + " is not a valid selectorType");
    }
}




// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }

    else if (descriptorType.compare("ORB") == 0)
    {

        extractor = cv::ORB::create();
    }

    else if (descriptorType.compare("AKAZE") == 0)
    {

        extractor = cv::AKAZE::create();

    }

    else if (descriptorType.compare("SIFT") == 0)
    {

        extractor = cv::xfeatures2d::SIFT::create();
    }

    else if (descriptorType.compare("FREAK") == 0)
    {
        extractor = cv::xfeatures2d::FREAK::create();

    }


    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    double t = (double)cv::getTickCount();
    float overlap = 0.0;
    for(size_t i = 0; i < dst_norm.rows; i++)
    {
        for(size_t j = 0 ; j < dst_norm.cols; j++)
        {
            int matrix_val = (int)dst_norm.at<float>(i,j);
            if(matrix_val > minResponse)
            {
                cv::KeyPoint curr_KP;
                curr_KP.pt = cv::Point(j,i);
                curr_KP.size = 2*apertureSize;
                curr_KP.response = matrix_val;

                bool b_overlap = false;
                for(auto it = keypoints.begin(); it!=keypoints.end(); ++it) //compare other KPs to current KP
                {
                    float kpt_overlap = cv::KeyPoint::overlap(curr_KP,*it);
                    if(kpt_overlap > overlap)
                    {
                        b_overlap = true;
                        if(curr_KP.response > (*it).response){
                            *it = curr_KP; //replace old KP with the current one
                            break;
                        }
                    }
                }
                if(!b_overlap)
                {
                    keypoints.emplace_back(curr_KP); // only add the KP if no overlap was found
                }
            }
        }
    }
    t = (cv::getTickCount()-t)/(cv::getTickFrequency());
    cout << "HARRIS detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    if(bVis) {
        string win_name = "Harris Corner Detection Results";
        cv::namedWindow(win_name, 5);
        cv::Mat visImage = dst_norm_scaled.clone();
        cv::drawKeypoints(dst_norm_scaled, keypoints, visImage, cv::Scalar::all(-1),
                          cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imshow(win_name, visImage);
        cv::waitKey(0);
    }
}


void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string &detectorType, bool bVis)
{

    double t = (double)cv::getTickCount();
    if(detectorType == "FAST")
    {
        auto detector = cv::FastFeatureDetector::create();
        detector->detect(img, keypoints);
    }
    else if(detectorType == "ORB")
   {
        auto detector = cv::ORB::create();
        detector->detect(img, keypoints);
   }
    else if(detectorType == "AKAZE")
   {
        auto detector = cv::AKAZE::create();
        detector->detect(img, keypoints);
   }
    else if(detectorType == "BRISK")
   {
        auto detector = cv::BRISK::create();
        detector->detect(img, keypoints);
   }
    else if(detectorType == "SIFT")
    {
        auto detector = cv::xfeatures2d::SIFT::create();
        detector->detect(img, keypoints);
    }
    else if(detectorType == "FREAK")
    {
        auto detector = cv::xfeatures2d::FREAK::create();
        detector->detect(img, keypoints);
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << detectorType << " Detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    if(bVis)
      {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "FAST Results";
        cv::namedWindow(windowName, 2);
        imshow(windowName, visImage);
        cv::waitKey(0);
      }
}

