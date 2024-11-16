// Create simple 3d highway enviroment using PCL for exploring self-driving car sensors

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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{

    Eigen::Vector4f minPoint(-10, -5, -2, 1.0);
    Eigen::Vector4f maxPoint(30, 8, 1, 1.0);

    // Eigen::Vector4f minPoint(-20, -6, -3, 1.0);
    // Eigen::Vector4f maxPoint(25, 6.5, 3, 1.0);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud= pointProcessorI->FilterCloud(inputCloud, 0.3, minPoint, maxPoint );
  	renderPointCloud(viewer,filterCloud,"filterCloud");

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> result = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2, "Custom");

    auto road = result.first;
    renderPointCloud(viewer, road, "road", Color(1,0,0));
    auto object = result.second;
    renderPointCloud(viewer, object, "object", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->EuclideanCluster(object, 0.9, 2, 50);
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(object, 1.0, 3, 30);
    int clusterId= 0;

    std::vector<Color> colors = {Color(1.0,0.0,0.0), Color(0.0,1.0,0.0), Color(0.0,0.0,1.0), Color(1.0,0.0,1.0), Color(0.0,1.0,1.0), Color(1.0,1.0,0.0)};
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters){
        //cout << "cluster size: ";
        //pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "objCloud"+std::to_string(clusterId), colors[clusterId]);
        ++clusterId;
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, Color(0.0,0.0,1.0), 1);
    }

}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor 
    Lidar *ldr = new Lidar(cars,0.0);
    //renderRays(viewer, ldr->position, ldr->scan());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = ldr->scan();
    //renderPointCloud(viewer, ldr->scan(), "hkn");
    // Create point processor
    ProcessPointClouds<pcl::PointXYZ> *pointProcessor(new ProcessPointClouds<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr road(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> result = pointProcessor->SegmentPlane(cloud, 100, 0.2, "Custom");

    road = result.first;
    renderPointCloud(viewer, road, "road", Color(1,0,0));
    object = result.second;
    renderPointCloud(viewer, object, "object", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(object, 1.0, 3, 30);
    int clusterId= 0;

    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters){
        cout << "cluster size: ";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "objCloud"+std::to_string(clusterId), colors[clusterId]);
        ++clusterId;
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId, Color(1,0,0), 1);

    }
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
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    //simpleHighway(viewer);
	// cityBlock(viewer);
    
    // while (!viewer->wasStopped ())
    // {
    //     viewer->spinOnce ();
    // } 

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
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }
    cout << "Simulate";
    return 0;
}








// void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
// {
// 	ProcessPointClouds<pcl::PointXYZI> *pointProcessorI (new ProcessPointClouds<pcl::PointXYZI>);

//     pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("src/sensors/data/pcd/data_1/0000000000.pcd");
//   	//renderPointCloud(viewer,inputCloud,"inputCloud");
//     Eigen::Vector4f minPoint(-10, -5, -2, 1.0);
//     Eigen::Vector4f maxPoint(30, 8, 1, 1.0);
//     pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud= pointProcessorI->FilterCloud(inputCloud, 0.3, minPoint, maxPoint );
//   	renderPointCloud(viewer,filterCloud,"filterCloud");

//     std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> result = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);

//     auto road = result.first;
//     renderPointCloud(viewer, road, "road", Color(1,0,0));
//     auto object = result.second;
//     renderPointCloud(viewer, object, "object", Color(0,1,0));

//     std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(object, 1.0, 3, 30);
//     int clusterId= 0;

//     std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1), Color(1,1,0)};
//     for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters){
//         cout << "cluster size: ";
//         pointProcessorI->numPoints(cluster);
//         renderPointCloud(viewer, cluster, "objCloud"+std::to_string(clusterId), colors[clusterId]);
//         ++clusterId;
//         Box box = pointProcessorI->BoundingBox(cluster);
//         renderBox(viewer, box, clusterId, Color(1,0,0), 1);
//     }

// }