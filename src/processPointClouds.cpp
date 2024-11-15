// PCL lib Functions for processing point clouds 
#include <boost/filesystem.hpp>
#include "processPointClouds.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>  // for toPCLPointCloud2 and fromPCLPointCloud2
#include <pcl/PCLPointCloud2.h>


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

    // // Fill in the function to do voxel grid point reduction and region based filtering
    // pcl::PCLPointCloud2::Ptr cloud2;
    // pcl::toPCLPointCloud2(*cloud, *cloud2);

    // pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
 
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
        << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

    // Create the filtering object
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT>);

    typename pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);



    // //pcl::PCLPointCloud2::Ptr cloud_filtered;
    // typename pcl::PointCloud<PointT>::Ptr cloudfilt;
    // pcl::fromPCLPointCloud2(*cloud_filtered, *cloudfilt);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    for (int index : inliers->indices){
        cloudInliers->points.push_back(cloud->points[index]);
    }
    // Extract the outliers (points that do not belong to the plane)
    extract.setNegative(true);  // Extract outliers
    extract.filter(*cloudOutliers);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold, std::string RANSACMethod)
{
     // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    if (RANSACMethod == "PCL"){
      // Create the segmentation object
      pcl::SACSegmentation<PointT> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setMaxIterations (maxIterations);
      seg.setDistanceThreshold (distanceThreshold);

      // Create the filtering object
      pcl::ExtractIndices<PointT> extract;
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud);
      seg.segment (*inliers, *coefficients);

  
      if (inliers->indices.size () == 0)
      	{
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        //break;
        }

      } else { 
        // RansacPlane returns std::unordered_set<int>, convert it to pcl::PointIndices::Ptr
        std::unordered_set<int> inlier_indices = this->RansacPlane(cloud, maxIterations, distanceThreshold);
        for (int index : inlier_indices) {
            inliers->indices.push_back(index);
        }
    }
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return this->SeparateClouds(inliers,cloud);
}


template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int indice, typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<int>& cluster, std::vector<bool>& isprocessed, KdTree* tree, float distanceTol)

{
    if (isprocessed[indice]==false){
        // Mark the current index as processed
        isprocessed[indice] = true;
        // Add the current index to the cluster
        cluster.push_back(indice);

        // Directly create a pcl::PointXYZI from the point in the cloud
        pcl::PointXYZI searchPoint = cloud->points[indice];

        // Search for nearest neighbors in the KdTree using the point directly from the cloud
        std::vector<int> nearest = tree->search(searchPoint, distanceTol);
        std::cout << "Nearest points found: " << nearest.size() << " for point index: " << indice << std::endl;

        // Iterate through each nearest neighbor
        for (int id : nearest)
        {
            // If the point has not been processed, call proximity recursively
            if (!isprocessed[id])
                proximity(id, cloud, cluster, isprocessed, tree, distanceTol);
        }
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::EuclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize)
{

    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
     // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.8, 0.8, 0.8);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*


    KdTree* tree = new KdTree;

    for (int i = 0; i < cloud_filtered->points.size(); ++i) {
        // Insert each point into the KdTree
        tree->insert(cloud_filtered->points[i], i);
    }
    
	// TODO: Fill out this function to return list of indices for each cluster
    auto start = std::chrono::steady_clock::now();
	std::vector<typename pcl::PointCloud<PointT>::Ptr>  clusters;

	std::vector<bool> isprocessed(cloud_filtered->points.size(), false);
    std::cout << cloud_filtered->points.size() << std::endl;

	int i = 0;
	while (i < cloud_filtered->points.size())
	{
		if (isprocessed[i])
		{
			i++;
			continue;

		}
		typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        std::vector<int> cluster_id;
		proximity(i, cloud_filtered, cluster_id, isprocessed, tree, distanceTol);
        std::cout << "Cluster ID Size: " << cluster_id.size() << std::endl;
        if (cluster_id.size() < maxSize && cluster_id.size() > minSize) {
            for (int i : cluster_id) {
                cluster->points.push_back(cloud_filtered->points[i]);
                cluster->width = cluster->points.size();
                cluster->height = 1;
            }
            clusters.push_back(cluster);
  		}
		i++;
	}
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Euclidean Clustering took " << duration.count() << " milliseconds\n";
 
	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
     // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (clusterTolerance, clusterTolerance, clusterTolerance);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

    // typename std::pair<typename pcl::PointCloud<PointT>::Ptr,typename pcl::PointCloud<PointT>::Ptr> result = this->SegmentPlane(cloud, 100, 0.2);
    // cloud_filtered = result.second;
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_filtered);

    // KdTree* tree = new KdTree;
  
    // for (int i=0; i<cloud->points.size(); i++) 
    // 	tree->insert(cloud->points[i],i); 

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    for (const auto& cluster : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (const auto& idx : cluster.indices) {
            cloud_cluster->push_back((*cloud_filtered)[idx]);
        } //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }
    // Insert the point cloud into the vector


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

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    for (int i = 0; i < maxIterations; i++) {
        std::unordered_set<int> temp;

        // Randomly sample three points
        int firstpoint_line = rand() % cloud->width;
        int secondpoint_line = rand() % cloud->width;
        int thirdpoint_line = rand() % cloud->width;

        // Retrieve the selected points
        float x1 = cloud->points[firstpoint_line].x;
        float y1 = cloud->points[firstpoint_line].y;
        float z1 = cloud->points[firstpoint_line].z;

        float x2 = cloud->points[secondpoint_line].x;
        float y2 = cloud->points[secondpoint_line].y;
        float z2 = cloud->points[secondpoint_line].z;

        float x3 = cloud->points[thirdpoint_line].x;
        float y3 = cloud->points[thirdpoint_line].y;
        float z3 = cloud->points[thirdpoint_line].z;

        // Calculate the plane coefficients
        auto cross_prod = [&] () -> std::array<float, 3> {
            return {(y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1),
                    (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1),
                    (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)};
        };

        float A_coeff = cross_prod()[0];
        float B_coeff = cross_prod()[1];
        float C_coeff = cross_prod()[2];
        float D_coeff = -(A_coeff * x1 + B_coeff * y1 + C_coeff * z1);

        float norm = sqrt(A_coeff * A_coeff + B_coeff * B_coeff + C_coeff * C_coeff);

        // Measure distance between every point and the plane
        for (int j = 0; j < cloud->width; j++) {
            float X = cloud->points[j].x;
            float Y = cloud->points[j].y;
            float Z = cloud->points[j].z;

            float d = fabs(A_coeff * X + B_coeff * Y + C_coeff * Z + D_coeff) / norm;

            // If the point is within the tolerance, consider it as an inlier
            if (d < distanceTol) {
                temp.insert(j);
            }
        }

        if (temp.size() > inliersResult.size()) {
            inliersResult = temp;
        }
    }

    return inliersResult;
}
