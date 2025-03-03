#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

// helper function to get distance to lidar cloud with filtering out outlier points
float getLidarPointCloudDistance(std::vector<LidarPoint> &lidarPoints)
{
	std::vector<float> x_distances;
	x_distances.reserve(lidarPoints.size());
	float x_min = 1e8; // fallback value for too few lidar points
	for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
	{
		float x = it1->x;
		if (x_min > x) x_min = x;
		x_distances.push_back(x);
	}
	std::sort(x_distances.begin(), x_distances.end());
	int number_of_closest_points_to_consider = 9;
	double x_min_median = x_min; // distance after filtering out outlier lidar points (test for lidar based TTC calculation)
	if (x_distances.size() > number_of_closest_points_to_consider)
	{
		// median of odd number of closest points
		x_min_median = x_distances[number_of_closest_points_to_consider / 2];
	}
	return x_min_median;
}

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait, int nFrameCounter)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        //cv::RNG rng(it1->boxID);
		cv::RNG rng(it1->lidarPoints.empty()?0:it1->lidarPoints.front().x*0x2000000);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;

        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-(xw-6) * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }
		
		double xwmin_median = getLidarPointCloudDistance(it1->lidarPoints);
		
        // draw enclosing rectangle
        //cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);
		// find height of median dist:
		float z_of_closest_point = 0;
		for (auto &p : it1->lidarPoints) if (p.x == xwmin_median) z_of_closest_point = p.z;
        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250* imageSize.width /2000, bottom+50* imageSize.height / 2000), cv::FONT_ITALIC, imageSize.height/1000.0, currColor);
        sprintf(str2, "xmin=%2.2f m (median %2.2f, z=%2.2f), yw=%2.2f m", xwmin, xwmin_median, z_of_closest_point, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250* imageSize.width / 2000, bottom+125 * imageSize.height / 2000), cv::FONT_ITALIC, imageSize.height / 1000.0, currColor);
		
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);
	{
		// save image
		char tmp[20];
		sprintf(tmp, "top_%02d.png", nFrameCounter);
		cv::imwrite(tmp, topviewImg);
	}


    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
	std::vector<std::pair<cv::DMatch, float> > PreFilteredMatchesWithDistance;
	float distance_sum = 0;
	for (auto &match : kptMatches)
	{
		cv::KeyPoint &prevKeyPoint = kptsPrev[match.queryIdx];
		cv::KeyPoint &currKeyPoint = kptsCurr[match.trainIdx];
		if (boundingBox.roi.contains(currKeyPoint.pt))
		{
			auto dist_vect = currKeyPoint.pt - prevKeyPoint.pt;
			float dist = sqrt(dist_vect.x*dist_vect.x + dist_vect.y*dist_vect.y);
			distance_sum += dist;
			PreFilteredMatchesWithDistance.push_back(std::pair<cv::DMatch, float>(match, dist));
		}
	}
	float mean_distance = distance_sum / PreFilteredMatchesWithDistance.size();
	float max_distance = mean_distance * 2 + 1; // with 1-1 pixel shift multiplication only is not reliable, I allowed +1 for pixel coordinate rounding error
	for (auto &match_with_dist : PreFilteredMatchesWithDistance)
	{
		float dist = match_with_dist.second;
		if (dist <= max_distance)
		{
			boundingBox.kptMatches.push_back(match_with_dist.first);
			boundingBox.keypoints.push_back(kptsCurr[match_with_dist.first.trainIdx]); // not used anywhere
		}
	}
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> &kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // for each point pairs, store the distance for previous and current frame
	std::vector<std::pair<float, float> > distance_pairs;
	float max_point_dist = 0; // we'll filter out close point pairs based on this
	for (auto match_it1 = kptMatches.begin(); match_it1!=kptMatches.end();++match_it1)
	{
		auto kp1_prev = kptsPrev[match_it1->queryIdx].pt;
		auto kp1_curr = kptsCurr[match_it1->trainIdx].pt;
		if (visImg != nullptr)
		{
			cv::line(*visImg, kp1_prev, kp1_curr, cv::Scalar(255, 255, 0), 1);
		}
		for (auto match_it2 = match_it1 + 1; match_it2 != kptMatches.end(); ++match_it2)
		{
			auto kp2_prev = kptsPrev[match_it2->queryIdx].pt;
			auto kp2_curr = kptsCurr[match_it2->trainIdx].pt;
			auto prev_vect = kp2_prev - kp1_prev;
			float dist_prev = sqrt(prev_vect.dot(prev_vect));
			auto curr_vect = kp2_curr - kp1_curr;
			float dist_curr = sqrt(curr_vect.dot(curr_vect));
			distance_pairs.push_back(std::pair<float, float>(dist_prev, dist_curr));
			if (dist_curr > max_point_dist) max_point_dist = dist_curr;
		}
	}
	std::vector<float> filtered_distance_ratios;
	for (auto dist_pair : distance_pairs)
	{
		if (dist_pair.first>0 && dist_pair.second > max_point_dist / 2)
		{
			filtered_distance_ratios.push_back(dist_pair.second / dist_pair.first);
		}
	}
	// nth_element is more optimal than sort, we only need the middle value (or values if length is even)
	std::nth_element(filtered_distance_ratios.begin(),
		filtered_distance_ratios.begin() + filtered_distance_ratios.size() / 2,
		filtered_distance_ratios.end());
	float median_dist_ratio = filtered_distance_ratios[filtered_distance_ratios.size() / 2];
	if (filtered_distance_ratios.size() % 2 == 0)
	{
		int idx = filtered_distance_ratios.size() / 2 - 1;
		std::nth_element(filtered_distance_ratios.begin(),
			filtered_distance_ratios.begin() + idx,
			filtered_distance_ratios.end());
		median_dist_ratio = (median_dist_ratio + filtered_distance_ratios[idx]) / 2;
	}
	// using median instead of mean, big errors have less effect in the result
	
	TTC = 1 / (frameRate * (median_dist_ratio - 1)); // frameRate is 1/s
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
	float x_min_prev = getLidarPointCloudDistance(lidarPointsPrev);
	float x_min_curr = getLidarPointCloudDistance(lidarPointsCurr);
	if (x_min_curr >= x_min_prev) TTC = 1000;
	else
	{
		float x_dist = x_min_prev - x_min_curr;
		float x_relative_velocity = x_dist * frameRate; // frameRate is in 1/s
		TTC = x_min_curr / x_relative_velocity;
	}
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
	std::map<int, std::map<int, int> > bounding_box_matches;
	// bounding_box_matches maps prev frame bounding box id to all possible candidates
	// in current frame: a map from candidate box id to number of keypoint matches
	for (auto &match : matches)
	{
		cv::KeyPoint &prevKeyPoint = prevFrame.keypoints[match.queryIdx];
		cv::KeyPoint &currKeyPoint = currFrame.keypoints[match.trainIdx];
		for (auto &prevBox : prevFrame.boundingBoxes)
		{
			if (prevBox.roi.contains(prevKeyPoint.pt))
			{
				for (auto &currBox : currFrame.boundingBoxes)
				{
					if (currBox.roi.contains(currKeyPoint.pt))
					{
						bounding_box_matches[prevBox.boxID][currBox.boxID]++;
					}
				}
			}
		}
	}
	// search for max match count for each box in prevFrame:
	for (auto &p : bounding_box_matches)
	{
		int max_match = 0;
		int max_match_id = -1;
		for (auto p2 : p.second)
		{
			if (p2.second > max_match)
			{
				max_match = p2.second;
				max_match_id = p2.first;
			}
		}
		if (max_match_id != -1)
		{
			bbBestMatches[p.first] = max_match_id;
		}
	}
}
