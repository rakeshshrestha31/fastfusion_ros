//
// Created by rakesh on 20/02/18.
//

#ifndef FASTFUSION_META_ROS_PANGOLIN_VIEWER_H
#define FASTFUSION_META_ROS_PANGOLIN_VIEWER_H

#include <pcl/PolygonMesh.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include <mutex>
#include <atomic>

#include <camerautils/camerautils.hpp>

namespace fastfusion_node {

/**
 * @brief Mesh visualizer using pangolin library
 */
class PangolinViewer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   *
   * @brief constructor to initialize the viewer
   * @todo read a config file and setup viewer based on that
   */
  PangolinViewer();
  /**
   *
   * @brief set mesh to visualize
   * @param mesh the mesh to visualize
   */
  void setMesh(boost::shared_ptr<pcl::PolygonMesh> mesh);

  /**
   * @brief main function that visualizes the mesh
   */
  void run();

  /**
   * @brief sets the bUpdateMesh_ flag to true so that run() function updates the visualizer
   */
  void updateMesh() { bUpdateMesh_ = true; }

  /**
   *
   * @param cameraInfo info of the current camera (only pose is used)
   */
  void updateCameraPose(CameraInfo &cameraInfo);

protected:
  boost::shared_ptr<pcl::PolygonMesh> mesh_;
  std::mutex meshMutex_;
  std::mutex cameraPoseMutex_;
  std::atomic_bool bUpdateMesh_;

  Eigen::Matrix4f cameraPose_;

  float imageWidth_, imageHeight_;
  float viewpointX_, viewpointY_, viewpointZ_, viewpointF_;
  float cameraSize_;
  float cameraLineWidth_;

  void drawCameraFrustum();
};

} // namespace fastfusion_node

#endif //FASTFUSION_META_ROS_PANGOLIN_VIEWER_H
