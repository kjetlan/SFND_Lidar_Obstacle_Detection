
#include "ransac.h"

template<typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	// Hold indexes of inliners with best fit over all iterations
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	int numPoints = cloud->points.size();

	// For max iterations 
	for (int i = 0; i < maxIterations; i++)
	{
		// Hold indexes of inliners in current interation
		std::unordered_set<int> inliers;

		// Randomly sample subset of point cloud
		// - For line need minimum 2 points
		// - For plane need minimum 3 points

		while(inliers.size() < 3) {
			// Note: "rand() % N" gives biased random sampling
			// FIXME: Replace with "std::uniform_int_distribution" or similar
			inliers.insert(rand() % numPoints);
		}
		
		// Extract points based on the random indecies
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = inliers.cbegin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		// Equation of line: Ax + By + C = 0
		// See: https://en.wikipedia.org/wiki/Linear_equation#Two-point_form
		// float A = y1 - y2;
		// float B = x2 - x1;
		// float C = x1*y2 - x2*y1;

		// Equation of a plane: Ax + By + Cz + D = 0
		// Defined by the cross product of two vectors in the plane
		// v1 = p2 - p1
		// v2 = p3 - p1
		// vn = v1 x v2 (Normal Vector)
		float a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		float b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		float d = -(a*x1 + b*y1 + c*z1);
		float denominator = sqrt(a*a + b*b + c*c);

		// Measure distance between every point and fitted plane
		// If distance is smaller than threshold count it as inlier
		for (int index = 0; index < numPoints; index++)
		{
			// We can skip points that are already inliers, like the points defining the plane
			if (inliers.count(index) > 0) {
				continue;
			}

			PointT point = cloud->points[index];
			float distance = fabs(a*point.x + b*point.y + c*point.z + d) / denominator;
			
			if (distance <= distanceTol)
			{
				inliers.insert(index);
			}
		}

		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}
	
	return inliersResult;

}
