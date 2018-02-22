//
// Created by rakesh on 20/02/18.
//

#include <fastfusion_node/pangolin_viewer.h>
#include <chrono>
#include <thread>

namespace fastfusion_node {

PangolinViewer::PangolinViewer() :
  bUpdateMesh_(false), bColorEnabled_(true), bDisplayMode_(1),
  bTerminate_(false), currentNF_(0), currentNV_(0) {
  imageWidth_ = 640;
  imageHeight_ = 480;

  viewpointX_ = 0;
  viewpointY_ = -0.7;
  viewpointZ_ = -1.8;
  viewpointF_ = 500;

  cameraSize_ = 0.16;
  cameraLineWidth_ = 3;

  cameraPose_.setIdentity();
}

void PangolinViewer::setMesh(MeshInterleaved *mesh) {
  std::unique_lock<std::mutex> lock(meshMutex_);
  meshInterleaved_ = mesh;
  meshDrawPointer_ = boost::shared_ptr<PointerMeshDraw>(new PointerMeshDraw(*mesh, bLightingEnabled_));
  bUpdateMesh_ = true;
}

void PangolinViewer::drawMesh() {
  if (bLightingEnabled_) {
    drawMeshPointer();
  } else {
    drawMeshInterleaved();
  }
}

void PangolinViewer::generateBuffers() {
  glewInit();
  glGenBuffers(1, &vertexBuffer_);
  glGenBuffers(1, &faceBuffer_);
  glGenBuffers(1, &colorBuffer_);
  glGenBuffers(1, &normalBuffer_);
  buffersGenerated_ = true;
}


void PangolinViewer::drawMeshPointer() {
  std::unique_lock<std::mutex> lock(meshMutex_);

  if (!meshDrawPointer_ || !meshDrawPointer_->nf) {
    return;
  }
  if (!buffersGenerated_){
    generateBuffers();
  }

  if (meshDrawPointer_->nv != currentNV_ || meshDrawPointer_->nf != currentNF_) {
    currentNF_ = meshDrawPointer_->nf;
    currentNV_ = meshDrawPointer_->nv;

    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer_);
    glBufferData(GL_ARRAY_BUFFER, meshDrawPointer_->nv * 3 * sizeof(float), meshDrawPointer_->v, GL_STATIC_DRAW);
    if (meshDrawPointer_->colored) {
      glBindBuffer(GL_ARRAY_BUFFER, colorBuffer_);
      glBufferData(GL_ARRAY_BUFFER, meshDrawPointer_->nv * 3, meshDrawPointer_->c, GL_STATIC_DRAW);
    }
    if (meshDrawPointer_->type == 1) {
      glBindBuffer(GL_ARRAY_BUFFER, normalBuffer_);
      glBufferData(GL_ARRAY_BUFFER, meshDrawPointer_->nn * 3 * sizeof(float), meshDrawPointer_->n, GL_STATIC_DRAW);
    }
    if (meshDrawPointer_->type != 1 && meshDrawPointer_->nf) {
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, faceBuffer_);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, meshDrawPointer_->nf * sizeof(unsigned int),
                   meshDrawPointer_->f, GL_STATIC_DRAW);
    }
  }
  glBindBuffer(GL_ARRAY_BUFFER,vertexBuffer_);
  glVertexPointer(3, GL_FLOAT, 0, 0);
  if(meshDrawPointer_->colored){
    glBindBuffer(GL_ARRAY_BUFFER,colorBuffer_);
    glColorPointer(3, GL_UNSIGNED_BYTE, 0, 0);
  }
  else{
    glColor3f(0.5f,0.5f,0.5f);
  }
  if(meshDrawPointer_->type==1 && meshDrawPointer_->nn>0){
    glBindBuffer(GL_ARRAY_BUFFER,normalBuffer_);
    glNormalPointer(GL_FLOAT, 0,0);
  }

  glEnableClientState(GL_VERTEX_ARRAY);
  if(bColorEnabled_) {
    glEnableClientState(GL_COLOR_ARRAY);
  }
  else{
    glColor3f(0.5f,0.5f,0.5f);
  }
  if(meshDrawPointer_->type==1 && meshDrawPointer_->nn>0){
    if(bColorEnabled_) glEnableClientState(GL_NORMAL_ARRAY);
  }

  if(bDisplayMode_==1){
    glPolygonMode(GL_FRONT, GL_LINE);
    glPolygonMode(GL_BACK, GL_LINE);
    glLineWidth(0.5f);
  }
  else{
    glPolygonMode(GL_FRONT, GL_FILL);
    glPolygonMode(GL_BACK, GL_FILL);
  }


  if(meshDrawPointer_->type==1){
    if(bDisplayMode_==2){
      fprintf(stderr,"\nDrawing Points");
      glDrawArrays(GL_POINTS, 0,meshDrawPointer_->nv);
    }
    else{
      fprintf(stderr,"\nDrawing Triangles");
      glDrawArrays(GL_TRIANGLES, 0,meshDrawPointer_->nv);
    }
  }
  else{
    if(bDisplayMode_==2){
      fprintf(stderr,"\nDrawing Points");
      glDrawArrays(GL_POINTS, 0,meshDrawPointer_->nv);
    }
    else{
      fprintf(stderr,"\nDrawing Triangles");
      glDrawElements(GL_TRIANGLES, meshDrawPointer_->nf, GL_UNSIGNED_INT,0);
    }
  }
  if(bColorEnabled_) glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
}

void PangolinViewer::drawMeshInterleaved() {
  std::unique_lock<std::mutex> lock(meshMutex_);

  if (!meshInterleaved_ || !meshInterleaved_->faces.size()) {
    return;
  }
//  glColor3f(1,1,1);
  if (!buffersGenerated_){
    generateBuffers();
  }

  if (meshInterleaved_->vertices.size() != currentNV_ || meshInterleaved_->faces.size() != currentNF_) {
    currentNF_ = meshInterleaved_->faces.size();
    currentNV_ = meshInterleaved_->vertices.size();

    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer_);
    glBufferData(GL_ARRAY_BUFFER, meshInterleaved_->vertices.size() * 3 * sizeof(float), meshInterleaved_->vertices.data(), GL_STATIC_DRAW);
    if (meshInterleaved_->colors.size()) {
      glBindBuffer(GL_ARRAY_BUFFER, colorBuffer_);
      glBufferData(GL_ARRAY_BUFFER, meshInterleaved_->vertices.size() * 3, meshInterleaved_->colors.data(), GL_STATIC_DRAW);
    }

    if (meshInterleaved_->faces.size()) {
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, faceBuffer_);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, meshInterleaved_->faces.size() * sizeof(unsigned int),
                   meshInterleaved_->faces.data(), GL_STATIC_DRAW);
    }
  }
  glBindBuffer(GL_ARRAY_BUFFER,vertexBuffer_);
  glVertexPointer(3, GL_FLOAT, 0, 0);
  if(meshInterleaved_->colors.size()) {
    glBindBuffer(GL_ARRAY_BUFFER,colorBuffer_);
    glColorPointer(3, GL_UNSIGNED_BYTE, 0, 0);
  }
  else {
    glColor3f(0.5f,0.5f,0.5f);
  }

  glEnableClientState(GL_VERTEX_ARRAY);
  if(bColorEnabled_) {
    glEnableClientState(GL_COLOR_ARRAY);
  }
  else{
    glColor3f(0.5f,0.5f,0.5f);
  }

  if(bDisplayMode_==1) {
    glPolygonMode(GL_FRONT, GL_LINE);
    glPolygonMode(GL_BACK, GL_LINE);
    glLineWidth(0.5f);
  } else {
    glPolygonMode(GL_FRONT, GL_FILL);
    glPolygonMode(GL_BACK, GL_FILL);
  }


  if (bDisplayMode_==2){
    glPointSize(2.0);
    glBindBuffer(GL_ARRAY_BUFFER,vertexBuffer_);
    glDrawArrays(GL_POINTS,0,meshInterleaved_->vertices.size());
  }
  else{
    glDrawElements(GL_TRIANGLES, meshInterleaved_->faces.size(), GL_UNSIGNED_INT,0);
  }
  if(bColorEnabled_) glDisableClientState(GL_COLOR_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);
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

void PangolinViewer::drawOriginFrame() {
  glLineWidth(cameraLineWidth_);
  glBegin(GL_LINES);

  glColor3f(1.0f, 0.0f, 0.0f);
  glVertex3f(0,0,0);
  glVertex3f(1, 0, 0);

  glColor3f(0.0f, 1.0f, 0.0f);
  glVertex3f(0,0,0);
  glVertex3f(0, 1, 0);

  glColor3f(0.0f, 0.0f, 1.0f);
  glVertex3f(0,0,0);
  glVertex3f(0, 0, 1);

  glEnd();
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
  pangolin::CreateWindowAndBind("Mesh Viewer", 1024, 768);
  generateBuffers();
  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
  pangolin::Var<bool> menuWireFrame("menu.Wireframe", true, true);
  pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
  pangolin::Var<bool> menuLighting("menu.Lighting", true, true);
  pangolin::Var<bool> menuColor("menu.Color", true, true);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024,768,viewpointF_,viewpointF_,512,389,0.1,1000),
    pangolin::ModelViewLookAt(viewpointX_,viewpointY_,viewpointZ_, 0,0,0,0.0,-1.0, 0.0)
  );

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  bool bFollow = true;

  // debug
  bUpdateMesh_ = true;
  while (!bTerminate_) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if(menuFollowCamera && bFollow)
    {
      s_cam.Follow(cameraPose_);
    }
    else if(menuFollowCamera && !bFollow)
    {
      s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(viewpointX_,viewpointY_,viewpointZ_, 0,0,0,0.0,-1.0, 0.0));
      s_cam.Follow(cameraPose_);
      bFollow = true;
    }
    else if(!menuFollowCamera && bFollow)
    {
      bFollow = false;
    }

    if (menuLighting && !bLightingEnabled_) {
      bLightingEnabled_ = true;
    } else if (!menuLighting && bLightingEnabled_) {
      bLightingEnabled_ = false;
    }

    if (menuColor && !bColorEnabled_) {
      bColorEnabled_ = true;
    } else if (!menuColor && bColorEnabled_) {
      bColorEnabled_ = false;
    }

    d_cam.Activate(s_cam);
    glClearColor(1.0f,1.0f,1.0f,1.0f);
    drawCameraFrustum();
    drawOriginFrame();
    drawMesh();

    pangolin::FinishFrame();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

  }

}

} // namespace fastfusion_node