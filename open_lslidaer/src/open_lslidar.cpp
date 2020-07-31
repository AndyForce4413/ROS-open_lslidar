#include <ros/ros.h>
#include <iostream>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

boost::shared_ptr<pcl::visualization::CloudViewer> viewer;                 // Point cloud viewer object.
bool saveCloud(false);
unsigned int filesSaved = 0;  

// This function is called every time the device has new data.
void
grabberCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  // viewer->showCloud(cloud);
	if (! viewer->wasStopped())
		viewer->showCloud(cloud);

	if (saveCloud)
	{
    cout<<"saveCloud"<<endl;
		std::stringstream stream;
		stream << "inputCloud" << filesSaved << ".pcd";
		std::string filename = stream.str();
		if (pcl::io::savePCDFileASCII(filename, *cloud) == 0)
		{
			filesSaved++;
			cout << "Saved " << filename << "." << endl;
		}
		else PCL_ERROR("Problem saving %s.\n", filename.c_str());

		saveCloud = false;
	}
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>);
  
  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  pcl::fromPCLPointCloud2 (*cloud, *cloud_pcl);

  grabberCallback(cloud_pcl);
}

// For detecting when SPACE is pressed.
void
keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		saveCloud = true;
}

// Creates, initializes and returns a new viewer.
boost::shared_ptr<pcl::visualization::CloudViewer>
createViewer()
{
	boost::shared_ptr<pcl::visualization::CloudViewer> v
	(new pcl::visualization::CloudViewer("OpenNI viewer"));
	v->registerKeyboardCallback(keyboardEventOccurred);
	return (v);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("lslidar_point_cloud", 1, cloud_cb);
  viewer = createViewer();
  // Spin
  ros::spin ();
}