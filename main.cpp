#include <iostream>
#include <stdio.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Tracker.h"

#define WRITE_RESULT_FILE 1

using namespace std;
using namespace cv;

/*
	Global constants used as
	parameters for Hough circle
*/
static const int SIGMA = 2; 
static const double HOUGH_CIRCLE_PARA1 = 100;
static const double HOUGH_CIRCLE_PARA2 = 30;
static const float MIN_WHEEL_R = 50;
static const float MAX_WHEEL_R = 80;

int main()
{
	
	int num_of_wheels = 0;
	
	// cv::VidepCapture object for reading input video file
	VideoCapture cap("cars_passing_input.mp4");

	// Define the codec and create VideoWriter object. The output is stored in 'output_result.avi' file 
	// Define the fps to be equal to 5.  
	VideoWriter write_video("output_result.mp4", 0x7634706d, 5, Size(cap.get(CAP_PROP_FRAME_WIDTH) / 2.5, cap.get(CAP_PROP_FRAME_HEIGHT) / 2.5 ));

	// object for the tracker class
	Tracker tracker_obj;

	// width of the video frames
	int WIDTH = cap.get(CAP_PROP_FRAME_WIDTH);

	// cv::Mat containers used for storing frames
	Mat rgb_frame, gray_frame, blur_frame, cropped_frame;
	
	/*
		Vectors for storing number of circles detected in
		each frame and their centers and radii
	*/
	vector<Vec3f> circles;
	vector<Point>centers;
	vector<int>radius;

	// read frames till end of video file is reached
	while ((cap.get(CV_CAP_PROP_POS_FRAMES)) < cap.get(CV_CAP_PROP_FRAME_COUNT))
	{
		// read new frame
		cout << "Reading frame " << cap.get(CV_CAP_PROP_POS_FRAMES) << endl;
		cap >> rgb_frame;
		
		// crop video to get better view of the wheels
		cropped_frame = rgb_frame(Rect(0, 300, WIDTH, 400));

		// convert to gray scale
		cvtColor(cropped_frame, gray_frame, CV_BGR2GRAY);

		// Apply gaussian blur
		GaussianBlur(gray_frame, blur_frame, Size(10 * SIGMA - 1, 10 * SIGMA - 1), SIGMA, SIGMA);

		// Find wheels using Hough circles
		HoughCircles(blur_frame, circles, CV_HOUGH_GRADIENT, 1, WIDTH / 3, HOUGH_CIRCLE_PARA1, HOUGH_CIRCLE_PARA2, MIN_WHEEL_R, MAX_WHEEL_R);

		for (size_t i = 0; i < circles.size(); i++)
		{
			// x, y points of the center of the wheel
			Point center(cvRound(circles[i][0]), cvRound(circles[i][1] + 300));
			int rad = cvRound(circles[i][2]);

			radius.push_back(rad);
			centers.push_back(center);
		}

		// Pass the center cordinates and radius of each detected wheel 
		// for tracking the wheels
		tracker_obj.track_wheel(centers, radius);

		for (size_t i = 0; i < tracker_obj.tracked_wheel.size(); i++)
		{
			if (tracker_obj.tracked_wheel[i].detection_found)
			{
				// while the center of the tracked wheels is inside the frame - to avoid errors
				if ((tracker_obj.tracked_wheel[i].track_center.x) > 0 && (tracker_obj.tracked_wheel[i].track_center.y) > 0 &&
					(tracker_obj.tracked_wheel[i].track_center.x < rgb_frame.cols) && (tracker_obj.tracked_wheel[i].track_center.y < rgb_frame.rows))
				{
					// Draws circle around the detected wheel
					circle(rgb_frame, tracker_obj.tracked_wheel[i].track_center, tracker_obj.tracked_wheel[i].track_radius, Scalar(0, 255, 0), 3, 8, 0);

					putText(rgb_frame, to_string(tracker_obj.tracked_wheel[i].track_id), tracker_obj.tracked_wheel[i].track_center, FONT_HERSHEY_SIMPLEX, 1, 
																																	Scalar(0, 255, 255), 2, 8);

					// Point for printing the velocity of the wheel on the screen
					Point speed_loc(tracker_obj.tracked_wheel[i].track_center.x - tracker_obj.tracked_wheel[i].track_radius, 
																		tracker_obj.tracked_wheel[i].track_center.y + tracker_obj.tracked_wheel[i].track_radius);

					putText(rgb_frame, "pix/frame:" + to_string(tracker_obj.tracked_wheel[i].velocity), speed_loc, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 2, 8);
									
				}
			}	
			// Calculates the number of videos that have passed the camera
			int tmp_index = tracker_obj.tracked_wheel.size();
			num_of_wheels = tracker_obj.tracked_wheel[tmp_index - 1].track_id + 1;
		}

		putText(rgb_frame, "# of wheels:" + to_string(num_of_wheels), Point(2, 25),
			FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 255, 255), 2, 8);

		// view the resulting output
		if (!WRITE_RESULT_FILE)
		{
			// resize video - done to increase speed while viewing real time
			resize(rgb_frame, rgb_frame, Size(640, 480));
			imshow("Output", rgb_frame);

			waitKey(0);
		}
		// write the output into a video file
		else
		{
			cout << "Writing frame " << cap.get(CV_CAP_PROP_POS_FRAMES) << endl;

			// resize video - done to reduce size of the file created
			resize(rgb_frame, rgb_frame, Size(rgb_frame.cols / 2.5, rgb_frame.rows / 2.5));
			write_video.write(rgb_frame);
		}

		circles.clear();
		centers.clear();
		radius.clear();
	}

	cout << "EOF" << endl;
	// Release the VideoCapture and VideoWriter objects
	cap.release();
	write_video.release();

	return 0;
}