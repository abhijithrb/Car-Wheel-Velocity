#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>

using namespace std;
using namespace cv;

class Tracker
{
private:
	
	// ID assigned for a new detection
	int next_track_id = 0;
	
	/*
		Finds the smallest element in a vector and returms
		its index
	*/
	int indexofSmallestElement(vector<float> vector);
	
	/*
		Finds the Euclidean distance between two points
		in an image
	*/
	float euclideanDist(cv::Point a, cv::Point b);

public:
	/*
		Structure type with information about each tracked wheel
		- track_id: the ID number of each wheel
		- track_radius: Radius of detected wheel
		- track_center: x,y points of the center of the wheel
		- detection_found: Flag to indicate if a wheel was detected for the current frame
		- skipped_frames: number of frames a tracked wheel did not have new detections
		- velocity: velocity of the wheel in terms of pixels/frame
	*/
	struct tracks 
	{
		int track_id = 0;
		int track_radius = 0;
		Point track_center;
		bool detection_found;
		int skipped_frames = 0;
		float velocity = 0;
	};

	// vector of type tracks containing information about each wheel that is tracked
	vector<tracks>tracked_wheel;

	Tracker();
	~Tracker();

	/*
		Function that tracks a wheel when detected using a simple centroid tracking 
		algorithm where the distance between the center of a previous detection is
		compared with the center of a new detection
		The function also computes the velocity of the wheel in terms of pixels/frame
	*/
	void track_wheel(vector<Point>centers, vector<int>radius);
};