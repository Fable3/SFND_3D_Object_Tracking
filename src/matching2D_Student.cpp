#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = true;
	if (selectorType == "SEL_KNN") crossCheck = false; // crossCheck not supported for kNN matching in OpenCV
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = descriptorType=="DES_BINARY"? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
		if (descSource.type() != CV_32F)
		{
			// OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
			descSource.convertTo(descSource, CV_32F);
			descRef.convertTo(descRef, CV_32F);
		}
		matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

		// implement k-nearest-neighbor matching
		vector<vector<cv::DMatch>> knn_matches;
		
		matcher->knnMatch(descSource, descRef, knn_matches, 2); // finds the 2 best matches
		
		// filter matches using descriptor distance ratio test
		double minDescDistRatio = 0.8;
		for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
		{
			if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
			{
				matches.push_back((*it)[0]);
			}
		}
		cout << "# keypoints removed = " << knn_matches.size() - matches.size() << endl;
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
	else if (descriptorType.compare("FREAK") == 0)
	{
		extractor = cv::xfeatures2d::FREAK::create();
	}
	else if (descriptorType.compare("AKAZE") == 0)
	{
		extractor = cv::AKAZE::create();
	}
	else if (descriptorType.compare("SIFT") == 0)
	{
		extractor = cv::xfeatures2d::SIFT::create();
	}

    // perform feature description
    //double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    //t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    //cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasiOrHarris(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis, bool bUseHarris)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    //double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, bUseHarris, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    //t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    //cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

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


void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
	detKeypointsShiTomasiOrHarris(keypoints, img, bVis, false);
}


void cornernessHarris(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
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

	// locate local maxima in the Harris response matrix 
	// and perform a non-maximum suppression (NMS) in a local neighborhood around 
	// each maximum. 
	vector<cv::KeyPoint> keypoint_candidates;
	int x, y;
	int nms_size = apertureSize;
	for (x = 0; x < img.size().width; x++)
		for (y = 0; y < img.size().height; y++)
		{
			int response = dst_norm.at<float>(y, x);
			if (response > minResponse)
			{
				keypoint_candidates.emplace_back();
				cv::KeyPoint &kp = keypoint_candidates.back();
				kp.pt.x = x;
				kp.pt.y = y;
				kp.size = nms_size;
				kp.response = response;
			}
		}

	for (int idx = 0; idx < keypoint_candidates.size(); idx++)
	{
		auto &kp = keypoint_candidates[idx];
		if (kp.response == 0) continue;
		int idx2;
		vector<int> to_be_removed;
		for (idx2 = idx + 1; idx2 < keypoint_candidates.size(); idx2++)
		{
			auto &kp2 = keypoint_candidates[idx2];
			if (kp2.response == 0) continue;
			if (kp2.pt.x - nms_size * 2 < kp.pt.x && kp2.pt.x + nms_size * 2 > kp.pt.x &&
				kp2.pt.y - nms_size * 2 < kp.pt.y && kp2.pt.y + nms_size * 2 > kp.pt.y)
			{
				if (kp.response > kp2.response)
				{
					to_be_removed.push_back(idx2);
				}
				else
				{
					break;
				}
			}
		}
		if (idx2 == keypoint_candidates.size())
		{
			for (auto i : to_be_removed) keypoint_candidates[i].response = 0;
			keypoints.push_back(kp);
		}
	}

	if (bVis)
	{
		// visualize results
		string windowName = "Harris Corner Detector Response Matrix";
		cv::namedWindow(windowName, 4);
		cv::imshow(windowName, dst_norm_scaled);
		cv::waitKey(0);
	}
}


void detKeypointsHarris(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
	//detKeypointsShiTomasiOrHarris(keypoints, img, bVis, true);
	cornernessHarris(keypoints, img, bVis);
}

void detKeypointsHarrisWithGoodFeaturesToTrack(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
	detKeypointsShiTomasiOrHarris(keypoints, img, bVis, true);
}

void detKeypointsModern(vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
	cv::Ptr<cv::FeatureDetector> detector;
	if (detectorType == "FAST")
	{
		int nThreshold = 30; // intensity difference
		bool bNMS = true; // non maximum suppression
		detector = cv::FastFeatureDetector::create(nThreshold, bNMS, cv::FastFeatureDetector::TYPE_9_16);
	}
	else if (detectorType == "BRISK")
	{
		detector = cv::BRISK::create();
	}
	else if (detectorType == "ORB")
	{
		detector = cv::ORB::create(1000);
	}
	else if (detectorType == "AKAZE")
	{
		detector = cv::AKAZE::create();
	} else if (detectorType == "SIFT")
	{
		detector = cv::xfeatures2d::SIFT::create();
	}

	//double t = (double)cv::getTickCount();
	detector->detect(img, keypoints);
	//t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
	//cout << detectorType <<" with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
}
