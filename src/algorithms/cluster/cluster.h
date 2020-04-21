#ifndef CLUSTER_H_
#define CLUSTER_H_

#include "kdtree.h"

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize);

#endif /* CLUSTER_H_ */