#include "Tracker.h"

Tracker::Tracker()
{
	
}

int Tracker::indexofSmallestElement(vector<float> vector)
{
	int index = 0;

	for (int i = 1; i < vector.size(); i++)
	{
		if (vector[i] < vector[index])
			index = i;
	}

	return index;
}

float Tracker::euclideanDist(Point a, Point b)
{
	double res = norm(a - b);

	return res;
}

void Tracker::track_wheel(vector<Point>centers, vector<int>radius)
{
	// If there are no tracks i.e first wheel being tracked
	if (tracked_wheel.size() == 0) 
	{
		for (int idx = 0; idx < centers.size(); idx++)
		{
			tracked_wheel.push_back(tracks());

			tracked_wheel[idx].track_center = centers[idx];
			tracked_wheel[idx].track_radius = radius[idx];
			tracked_wheel[idx].track_id = next_track_id++;
			tracked_wheel[idx].detection_found = true;
			tracked_wheel[idx].skipped_frames = 0;
		}
	}
	else
	{
		int ID = 0;

		for (int idx = 0; idx < tracked_wheel.size(); idx++)
			// initialize for the new frame
			tracked_wheel[idx].detection_found = false;

		// For new detections and existing wheel
		if (centers.size() >= tracked_wheel.size()) 
		{
			vector<float>distances(centers.size(), 0);

			// update tracker with detected wheel
			for (int idx = 0; idx < tracked_wheel.size(); idx++)
			{
				// Find the Euclidean Distances between the centeres of existing tracked wheel and all new detected wheels
				for (int idy = 0; idy < centers.size(); idy++)
					distances[idy] = euclideanDist(centers[idy], tracked_wheel[idx].track_center);

				// Find which existing point is closest to new detection 
				ID = indexofSmallestElement(distances); 

				//number of pixels moved per frame
				tracked_wheel[idx].velocity = euclideanDist(centers[ID], tracked_wheel[idx].track_center); 

				// Update with new positions
				tracked_wheel[idx].track_center = centers[ID];
				tracked_wheel[idx].track_radius = radius[ID];
				tracked_wheel[idx].detection_found = true;
				tracked_wheel[idx].skipped_frames = 0;

				centers.erase(centers.begin() + ID);
			}

			// If any detection went unassigned, add the new detection to tracker. i.e new wheel added to tracker
			for (int idx = 0; idx < centers.size(); idx++)
			{
				tracked_wheel.push_back(tracks());

				int index = tracked_wheel.size() - 1;

				//number of pixels moved per frame
				tracked_wheel[index].velocity = euclideanDist(centers[idx], tracked_wheel[index].track_center); 

				// Update with new positions
				tracked_wheel[index].track_center = centers[idx];
				tracked_wheel[index].track_radius = radius[idx];
				tracked_wheel[index].track_id = next_track_id++;
				tracked_wheel[index].detection_found = true;
				tracked_wheel[index].skipped_frames = 0;
			}
		}
		// If a detection for existing track is not found
		else
		{
			vector<float>distances(tracked_wheel.size(), 0);

			// update tracker with detected wheels
			for (int idx = 0; idx < centers.size(); idx++)
			{
				// Find the Euclidean Distances between the centeres of existing wheels and all new detected wheels
				for (int idy = 0; idy < tracked_wheel.size(); idy++)
				{
					if (!tracked_wheel[idy].detection_found)
						distances[idy] = euclideanDist(centers[idx], tracked_wheel[idy].track_center);
				}

				// Find which existing wheel is closest to the new detection 
				ID = indexofSmallestElement(distances);

				//number of pixels moved per frame
				tracked_wheel[ID].velocity = euclideanDist(centers[idx], tracked_wheel[ID].track_center) / 1; 

				// Update with new positions
				tracked_wheel[ID].track_center = centers[idx];
				tracked_wheel[ID].track_radius = radius[idx];
				tracked_wheel[ID].detection_found = true;
				tracked_wheel[ID].skipped_frames = 0;
			}

			for (int idx = 0; idx < tracked_wheel.size(); idx++)
			{
				// if an existing tracked wheel has no new detection
				if (!tracked_wheel[idx].detection_found)
					// count the number of frames a detection was not found
					tracked_wheel[idx].skipped_frames++;
			}
		}
	}

	for (int idx = 0; idx < tracked_wheel.size(); idx++)
	{
		// If track didn't get detects for more than 3 frames, remove it
		if (tracked_wheel[idx].skipped_frames > 1) 
		{
			tracked_wheel.erase(tracked_wheel.begin() + idx);
			idx--;
		}
	}
}

Tracker::~Tracker()
{
	tracked_wheel.clear();
}