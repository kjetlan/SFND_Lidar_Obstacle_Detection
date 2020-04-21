
#include "cluster.h"

void proximity(std::vector<bool> &isProcessed, int id, const std::vector<std::vector<float>>& points, KdTree* tree, std::vector<int> &cluster, float distanceTol)
{
	// Mark point as processed and add point to cluster
	isProcessed[id] = true;
	cluster.push_back(id);
	
	// Search for nearby points, returns ids of nearby point
	std::vector<int> nearbyIds = tree->search(points[id], distanceTol);
    
	// Iterate through each nearby point
	for (int nearybyId : nearbyIds)
	{
		// If point has not been processed
		if (!isProcessed[nearybyId])
		{
			// Proximity(nearby point, cluster)
			proximity(isProcessed, nearybyId, points, tree, cluster, distanceTol);
		}
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{

	// Return list of indices for each cluster
	std::vector<std::vector<int>> clusters;
	std::vector<bool> isProcessed(points.size(),false); // all elements false

	for (int id = 0; id < points.size(); id++)
	{
		if (!isProcessed[id])
		{
			// Create cluster
			std::vector<int> cluster;
			proximity(isProcessed, id, points, tree, cluster, distanceTol);
			if (minSize <= cluster.size() && cluster.size() <= maxSize)
			{
				clusters.push_back(cluster);
			}
		}
	}
 
	return clusters;
}
