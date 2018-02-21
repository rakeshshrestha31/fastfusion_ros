//
// Created by rakesh on 20/02/18.
//

#include <fastfusion_node/pangolin_viewer.h>
#include <pangolin/pangolin.h>
#include <chrono>
#include <thread>

namespace fastfusion_node {

PangolinViewer::PangolinViewer() : bUpdateMesh_(false) {
  imageWidth_ = 640;
  imageHeight_ = 480;

  viewpointX_ = 0;
  viewpointY_ = -0.7;
  viewpointZ_ = -1.8;
  viewpointF_ = 500;

  cameraSize_ = 0.08;
  cameraLineWidth_ = 3;

  cameraPose_.setIdentity();
}

void PangolinViewer::setMesh(boost::shared_ptr<pcl::PolygonMesh> mesh) {
  std::unique_lock<std::mutex> lock(meshMutex_);
  mesh_ = mesh;
}

void PangolinViewer::drawCameraFrustum()
{
  const float &w = cameraSize_;
  const float h = w*0.75;
  const float z = w*0.6;

  glPushMatrix();

  {
    std::unique_lock<std::mutex> lock(cameraPoseMutex_);
    glMultMatrixf(cameraPose_.data());
  }

  glLineWidth(cameraLineWidth_);
  glColor3f(0.0f,1.0f,0.0f);
  glBegin(GL_LINES);
  glVertex3f(0,0,0);
  glVertex3f(w,h,z);
  glVertex3f(0,0,0);
  glVertex3f(w,-h,z);
  glVertex3f(0,0,0);
  glVertex3f(-w,-h,z);
  glVertex3f(0,0,0);
  glVertex3f(-w,h,z);

  glVertex3f(w,h,z);
  glVertex3f(w,-h,z);

  glVertex3f(-w,h,z);
  glVertex3f(-w,-h,z);

  glVertex3f(-w,h,z);
  glVertex3f(w,h,z);

  glVertex3f(-w,-h,z);
  glVertex3f(w,-h,z);
  glEnd();

  glPopMatrix();
}

void PangolinViewer::updateCameraPose(CameraInfo &cameraInfo) {
  auto R_cv = cameraInfo.getRotation();
  auto t_cv = cameraInfo.getTranslation();

  std::unique_lock<std::mutex> lock(cameraPoseMutex_);

  cameraPose_(3, 3) = 1;
  for (int i = 0;i < 3; i++) {
    for(int j = 0; j<3;j++) {
      cameraPose_(i,j) = (float)R_cv.at<double>(i,j);
    }
  }
  cameraPose_(0, 3) = (float)t_cv.at<double>(0, 0);
  cameraPose_(1, 3) = (float)t_cv.at<double>(1, 0);
  cameraPose_(2, 3) = (float)t_cv.at<double>(2, 0);
}

void PangolinViewer::run() {
  bUpdateMesh_ = false;
  std::cout << "creating pangolin window..." << std::endl;
  pangolin::CreateWindowAndBind("Mesh viewer", 960, 720);
  std::cout << "created pangolin window" << std::endl;
//  // 3D Mouse handler requires depth testing to be enabled
//  glEnable(GL_DEPTH_TEST);
//
//  // Issue specific OpenGl we might need
//  glEnable (GL_BLEND);
//  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//
//  pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
//  pangolin::Var<bool> menuWireFrame("menu.Wireframe", true, true);
//  pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
//
//  // Define Camera Render Object (for view / scene browsing)
//  pangolin::OpenGlRenderState s_cam(
//    pangolin::ProjectionMatrix(1024,768,viewpointF_,viewpointF_,480,360,0.1,1000),
//    pangolin::ModelViewLookAt(viewpointX_,viewpointY_,viewpointZ_, 0,0,0,0.0,-1.0, 0.0)
//  );
//
//  // Add named OpenGL viewport to window and provide 3D Handler
//  pangolin::View& d_cam = pangolin::CreateDisplay()
//    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -960.0f/360.0f)
//    .SetHandler(new pangolin::Handler3D(s_cam));
//
//  bool bFollow = true;
//
//  while (true) {
//    if (bUpdateMesh_) {
//      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//      drawCameraFrustum();
//
//      pangolin::FinishFrame();
//      std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    }
//  }

}

} // namespace fastfusion_node