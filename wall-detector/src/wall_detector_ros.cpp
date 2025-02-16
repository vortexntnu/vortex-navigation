#include <wall_detector/wall_detector_ros.hpp>
#include <sstream>
#include <filesystem>

namespace vortex{
namespace wall_detector{


WallDetectorNode::WallDetectorNode(const rclcpp::NodeOptions & options) : Node("wall_detector_node", options)
{



} //Class wall detector

int WallDetectorNode::detect_wall()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 15;
  cloud->height = 15;
  cloud->points.resize (cloud->width * cloud->height);

  // Generate the data
  for (auto& point: *cloud)
  {
    point.x = 1.0;
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  // Set a few outliers
  (*cloud)[0].x = 2.0;
  (*cloud)[3].x = -2.0;
  (*cloud)[6].x = 4.0;

  std::cerr << "Point cloud data: " << cloud->size () << " points" << std::endl;
  for (const auto& point: *cloud)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  //Define axis for vertical planes 
  Eigen::Vector3f axis;
  axis << 1.0, 0.0, 0.0;

  seg.setAxis(axis); //Axis declared in hpp file
  seg.setEpsAngle(pcl::deg2rad(10.0)); //Allow up to 10 degree deviation

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (const auto& idx: inliers->indices)
    std::cerr << idx << "    " << cloud->points[idx].x << " "
                               << cloud->points[idx].y << " "
                               << cloud->points[idx].z << std::endl;
  return 0;

} // wall detector





} //namespace wall_detector
} // namespace vortex

