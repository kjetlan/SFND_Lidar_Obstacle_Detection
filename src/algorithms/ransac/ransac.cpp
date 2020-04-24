
#include "ransac.h"

template<typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	// Hold indexes of inliners with best fit over all iterations
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	int numPoints = cloud->points.size();

	// Hold count of best inliers and inliers in current iteration
	int inliersCountBest = 0;
	int inliersCount     = 0;
	
	// Hold parameter values of best model coefficients
	// For a plane 'ax + by + cz + d' there are 4 coefficients (a,b,c,d)
	std::vector<float> modelCoefficients(4);

	// For max iterations 
	// Randomly sample subset and fit model
	// Measure distance between every point and fitted model
	// If distance is smaller than threshold count it as inlier
	// Return indicies of inliers from fitted model with most inliers
	for (int i = 0; i < maxIterations; i++)
	{
		// Reset inlier count
		inliersCount = 0;

		// Randomly sample subset of point cloud
		// For a plane we need 3 random samples
		std::unordered_set<int> samples;
		while(samples.size() < 3) {
			// FIXME: "rand() % N" gives biased random sampling, use "std::uniform_int_distribution"
			samples.insert(rand() % numPoints);
		}
		
		// Extract points based on the random indecies
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = samples.cbegin();
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

		// Normalize model coefficients
		// This avoids using 'sqrt' and division in inner for-loop
		a = a / denominator;
		b = b / denominator;
		c = c / denominator;
		d = d / denominator;

		// Measure distance between every point and fitted plane
		// If distance is smaller than threshold count it as inlier
		for (int index = 0; index < numPoints; index++)
		{
			PointT point = cloud->points[index];
			float distance = fabs(a*point.x + b*point.y + c*point.z + d);
			
			if (distance <= distanceTol)
			{
				inliersCount++;
			}
			// Quit early, if not enough remaining points to produce better inlier
			if (numPoints - index + inliersCount < inliersCountBest)
			{
				break;
			}
		}

		// Update modelCoefficients if we found a better inlier
		if (inliersCount > inliersCountBest)
		{
			inliersCountBest = inliersCount;

			modelCoefficients[0] = a;
			modelCoefficients[1] = b;
			modelCoefficients[2] = c;
			modelCoefficients[3] = d;
		}
	}

	// Use the best model to re-calculate the inliers and add them to 'inliersResult'
	// -- This turns out to be more efficient than calculating a set 'inliers' on every iteration
	// -- and replacing 'inliersResult' with 'inliers' if size of 'inliers' is greater than 'inliersResult'
	float a = modelCoefficients[0];
	float b = modelCoefficients[1];
	float c = modelCoefficients[2];
	float d = modelCoefficients[3];
	for (int index = 0; index < numPoints; index++)
	{
		PointT point = cloud->points[index];
		float distance = fabs(a*point.x + b*point.y + c*point.z + d);
		
		if (distance <= distanceTol)
		{
			inliersResult.insert(index);
		}
	}

	return inliersResult;

}
