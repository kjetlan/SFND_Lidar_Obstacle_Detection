#ifndef RANSAC_H_
#define RANSAC_H_

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <vector>
#include <ctime>
#include <unordered_set>

template<typename PointT>
std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

#endif /* RANSAC_H_ */