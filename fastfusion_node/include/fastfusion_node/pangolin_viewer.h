//
// Created by rakesh on 20/02/18.
//

#ifndef FASTFUSION_META_ROS_PANGOLIN_VIEWER_H
#define FASTFUSION_META_ROS_PANGOLIN_VIEWER_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include <mutex>
#include <atomic>
#include <pangolin/pangolin.h>

#include <camerautils/camerautils.hpp>
#include <fusion/mesh.hpp>

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
  void setMesh(MeshInterleaved *mesh);

  /**
   * @brief main function that visualizes the mesh
   */
  void run();

  /**
   * @brief terminate visualizer
   */
  void terminate() { bTerminate_ = true; }

  /**
   *
   * @param cameraInfo info of the current camera (only pose is used)
   */
  void updateCameraPose(CameraInfo &cameraInfo);

protected:
  boost::shared_ptr<PointerMeshDraw> meshDrawPointer_;
  MeshInterleaved *meshInterleaved_;
  std::mutex meshMutex_;
  std::mutex cameraPoseMutex_;
  std::atomic_bool bUpdateMesh_;
  std::atomic_bool bTerminate_;
  bool bLightingEnabled_;
  bool bColorEnabled_;
  int bDisplayMode_;

  Eigen::Matrix4f cameraPose_;

  float imageWidth_, imageHeight_;
  float viewpointX_, viewpointY_, viewpointZ_, viewpointF_;
  float cameraSize_;
  float cameraLineWidth_;

  GLuint vertexBuffer_;
  GLuint faceBuffer_;
  GLuint edgeBuffer_;
  GLuint normalBuffer_;
  GLuint colorBuffer_;
  unsigned int vertexBufferSize_;
  unsigned int faceBufferSize_;
  unsigned int edgeBufferSize_;

  unsigned int currentNV_;
  unsigned int currentNF_;

  void generateBuffers();
  bool buffersGenerated_;

  // drawing helpers
  void drawCameraFrustum();
  void drawOriginFrame();
  void drawMesh();
  void drawMeshPointer();
  void drawMeshInterleaved();
};

} // namespace fastfusion_node

#endif //FASTFUSION_META_ROS_PANGOLIN_VIEWER_H
