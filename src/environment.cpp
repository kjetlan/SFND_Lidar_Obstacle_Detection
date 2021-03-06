/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

Color rgbColor(float r, float g, float b)
{
    return Color(r/255., g/255., b/255.);
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false; // if true: show cars and road
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor
    Lidar* lidar = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    // renderRays(viewer, lidar->position, cloud);
    renderPointCloud(viewer, cloud, "Point Cloud");

    // Create point processor
    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    // Segment point cloud into plane and obstacle
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(cloud, 100, 0.2);
    // renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1,0,0));
    // renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));


    // Clusters obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.5, 3, 30);

    // Render obstacle clusters
    int clusterId = 0;
    
    // Define Color Palette
    std::vector<Color> colors(5, Color(1,1,1));
    colors[0] = rgbColor(255,69,58); // Red
    colors[1] = rgbColor(50,215,75); // Green
    colors[2] = rgbColor(10,132,255); // Blue
    colors[3] = rgbColor(255,159,10); // Orange
    colors[4] = rgbColor(255,214,10); // Yellow
    

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstacleCloud"+std::to_string(clusterId),colors[clusterId % colors.size()]);

        // Add bounding box to cluster
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId,colors[clusterId % colors.size()],0.5);

        ++clusterId;
    }

}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    // For Debugging:
    // renderPointCloud(viewer,inputCloud,"inputCloud");
    // return;

    // Define Color Palette
    std::vector<Color> colors(7, Color(1,1,1));
    colors[0] = rgbColor(255,69,58); // Red
    colors[1] = rgbColor(50,215,75); // Green
    colors[2] = rgbColor(10,132,255); // Blue
    colors[3] = rgbColor(255,159,10); // Orange
    colors[4] = rgbColor(255,214,10); // Yellow
    colors[5] = rgbColor(191,90,242); // Purple
    colors[6] = rgbColor(152,152,157); // Gray
    Color brown = rgbColor(172,142,104); // Brown

    // Experiment with the values and find what works best
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.20, Eigen::Vector4f(-10, -5, -3, 1), Eigen::Vector4f(20, 7, 2, 1));
    // renderPointCloud(viewer,filterCloud,"filterCloud");

    // Segment point cloud into plane and obstacle
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
    // renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1,1,1));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", brown);


    // Clusters obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.4, 10, 800);

    // Render obstacle clusters
    int clusterId = 0;
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstacleCloud"+std::to_string(clusterId),colors[clusterId % colors.size()]);

        // Add bounding box to cluster
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId,colors[clusterId % colors.size()],0.2);

        ++clusterId;
    }

    // Render box of egoCar, used for finding car region to filter out in FilterCloud(..)
    // Box egoCarBox;
    // egoCarBox.x_max = 2.8;
    // egoCarBox.y_max = 1.5;
    // egoCarBox.z_max = 0;
    // egoCarBox.x_min = -2;
    // egoCarBox.y_min = -1.5;
    // egoCarBox.z_min = -1;
    // renderBox(viewer, egoCarBox, 0, colors[3], 0.2);
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    
    bool loadSingleImage = false; // Command line argument
    if (argc > 1 && argv[1])
    {
        loadSingleImage = true;
    }

    if (loadSingleImage)
    {
        // Single PCD image
        ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
        cityBlock(viewer, pointProcessorI, inputCloud);
        
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce ();
        } 
    }
    else
    {
        // Stream of PCD images
        ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
        std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
        auto streamIterator = stream.begin();
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
        
        while (!viewer->wasStopped ())
        {
            // Clear viewer
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();

            // Load pcd and run obstacle detection process
            inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
            cityBlock(viewer, pointProcessorI, inputCloudI);

            streamIterator++;
            if(streamIterator == stream.end())
            {
                streamIterator = stream.begin();
            }
            viewer->spinOnce();
        }
    }
}