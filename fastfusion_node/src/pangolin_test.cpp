//
// Created by rakesh on 21/02/18.
//

//#define USE_PCL_VISUALIZATION
#undef USE_PCL_VISUALIZATION

#include <X11/Xlib.h>
// bloody Xlib has a macro called Success that classes with Success constant in Eigen
#undef Success

#include <fastfusion_node/pangolin_viewer.h>
#include <boost/shared_ptr.hpp>
#include <thread>
using namespace fastfusion_node;
int main(int argc, char ** argv) {
  XInitThreads();
  boost::shared_ptr<PangolinViewer> pangolin_viewer(new PangolinViewer());
  auto thread = new std::thread(&PangolinViewer::run, pangolin_viewer.get());
  thread->detach();
  while (true);
  return 0;
}