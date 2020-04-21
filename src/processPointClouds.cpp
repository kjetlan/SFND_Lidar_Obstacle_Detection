// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

// Algorithms
#include "algorithms/ransac/ransac.h"
#include "algorithms/ransac/ransac.cpp" // using templates for ransac so also include .cpp to help linker
#include "algorithms/cluster/cluster.h"
#include "algorithms/cluster/kdtree.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filterCloud(new pcl::PointCloud<PointT>);

    // Voxel grid filtering
    typename pcl::VoxelGrid<PointT> voxelFilter;
    voxelFilter.setInputCloud(cloud);
    voxelFilter.setLeafSize(filterRes, filterRes, filterRes); // e.g. filterRes = 0.01 is 1cm leaf size
    voxelFilter.filter(*filterCloud);

    // Region based filtering (field-of-view)
    typename pcl::CropBox<PointT> fovFilter;
    fovFilter.setInputCloud(filterCloud);
    fovFilter.setMin(minPoint);
    fovFilter.setMax(maxPoint);
    fovFilter.filter(*filterCloud);

    // Filter egoCar from point cloud
    bool shouldFilterEgoCar = true;
    if (shouldFilterEgoCar)
    {
        // Hold indices of points belonging to egoCar
        pcl::PointIndices::Ptr egoCar(new pcl::PointIndices());

        // Find indices inside egoCar region
        typename pcl::CropBox<PointT> carFilter;
        carFilter.setInputCloud(filterCloud);
        carFilter.setMax(Eigen::Vector4f(2.8, 1.5, 0, 1));  // Points found based on data
        carFilter.setMin(Eigen::Vector4f(-2, -1.5, -1, 1)); // Points found based on data
        carFilter.filter(egoCar->indices);

        // Extract the egoCar from point cloud
        typename pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(filterCloud);
        extract.setIndices(egoCar);
        extract.setNegative(true);
        extract.filter(*filterCloud);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::cerr << "reduced to " << filterCloud->points.size() << " data points" << std::endl;

    return filterCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>);

    // Create an extraction object
    typename pcl::ExtractIndices<PointT> extract;

    // Extract the inliers to planeCloud
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*planeCloud);

    // Extract the outliers to obstacleCloud
    extract.setNegative(true); // inverted behavior
    extract.filter(*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Find inliers for the cloud
    bool useBuiltInMethod = false;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    if (useBuiltInMethod) {
        // Use Ransac method from PCL
        // See: http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

        // Create the segmentation object
        typename pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true); // Optional
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(maxIterations);
        seg.setDistanceThreshold(distanceThreshold);

        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
    } else {
        // Use custom Ransac method
        std::unordered_set<int> inliersResult = Ransac<PointT>(cloud, maxIterations, distanceThreshold);

        inliers->indices.clear();
        for (auto inlier : inliersResult) {
            inliers->indices.push_back(inlier);
        }
    }

    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Perform euclidean clustering to group detected obstacles
    bool useBuiltInMethod = false;
    if (useBuiltInMethod)
    {
        // See: http://pointclouds.org/documentation/tutorials/cluster_extraction.php    
        
        // Create the KdTree object for the search method of the extraction
        typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud);

        // 'clusterIndices' is a vector containing one instance of PointIndices for each detected cluster.
        std::vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(clusterTolerance);
        ec.setMinClusterSize(minSize);
        ec.setMaxClusterSize(maxSize);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(clusterIndices);

        // Use 'clusterIndices' to add each obstacle cluster to the 'clusters' vector
        for (pcl::PointIndices pointIndices : clusterIndices)
        {
            typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
            for (int index : pointIndices.indices)
            {
                cloudCluster->points.push_back(cloud->points[index]);
            }
            cloudCluster->width = cloudCluster->points.size();
            cloudCluster->height = 1;
            cloudCluster->is_dense = true;

            clusters.push_back(cloudCluster);
        }
    }
    else
    {
        // Create the KdTree object for the search method of the extraction
        KdTree* tree = new KdTree;
        std::vector<std::vector<float>> points;

        for (int index = 0; index < cloud->points.size(); index++)
        {
            PointT point = cloud->points[index];
            points.push_back({ point.x, point.y, point.z });
            
            tree->insert(points[index], index);
        }
                
        // 'clusterIndices' is a vector containing a list of indices for each detected cluster.
        std::vector<std::vector<int>> clusterIndices = euclideanCluster(points, tree, clusterTolerance);

        // Use 'clusterIndices' to add each obstacle cluster to the 'clusters' vector
        for (std::vector<int> pointIndices : clusterIndices)
        {
            typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
            for (int index : pointIndices)
            {
                cloudCluster->points.push_back(cloud->points[index]);
            }
            cloudCluster->width = cloudCluster->points.size();
            cloudCluster->height = 1;
            cloudCluster->is_dense = true;

            clusters.push_back(cloudCluster);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}