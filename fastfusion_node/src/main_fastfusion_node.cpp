/*
 * main_fastfusion_node.cpp
 *
 *  Created on: Sep 27, 2015
 *      Author: karrerm
 */
//#define USE_PCL_VISUALIZATION
#undef USE_PCL_VISUALIZATION

#include <X11/Xlib.h>
// bloody Xlib has a macro called Success that classes with Success constant in Eigen
#undef Success

#include <fastfusion_node/pangolin_viewer.h>
#include <boost/shared_ptr.hpp>
#include <thread>
#include "fastfusion_node/fastfusion_node.hpp"
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/surface/gp3.h>
//#include <pcl/visualization/pcl_visualizer.h>


/*
 * ROS-Node including the fastfusion mapping algorithm presented in
 * "Volumetric 3D Mapping in Real-Time on a CPU (Steinbrücker,Sturm, Cremers).
 * The node assumes depth and pose data is available
 */
using namespace fastfusion_node;
int main(int argc, char **argv)
{
  XInitThreads();

  ros::init(argc, argv, "fastfusion_node");

  ROS_INFO("\nStarting fastfusion node with name %s\n", ros::this_node::getName().c_str());

  FastFusionWrapper fastfusion;

  fastfusion.run();
  return 0;
}

