#include <class_loader/class_loader.h>
#include <utexas_planning/visualizers/no_visualization.h>

namespace utexas_planning {

  NoVisualization::~NoVisualization() {}
  void NoVisualization::init(int argc, char* argv[]) {}
  void NoVisualization::startEpisode(const State::ConstPtr& /* start_state */) {}
  void NoVisualization::updateState(const State::ConstPtr& /* state */, float /* timeout */) {}

} /* utexas_planning */

CLASS_LOADER_REGISTER_CLASS(utexas_planning::NoVisualization, utexas_planning::Visualizer);
