#include <class_loader/class_loader.h>
#include <utexas_planning/planners/random/random_planner.h>

namespace utexas_planning {

  RandomPlanner::~RandomPlanner() {}

  void RandomPlanner::init(const GenerativeModel::ConstPtr& model,
                           const YAML::Node& /* params */,
                           const std::string& /* output_directory */,
                           const boost::shared_ptr<RNG>& rng,
                           bool /* verbose */) {
    model_ = model;
    rng_ = rng;
  }

  void RandomPlanner::performEpisodeStartProcessing(const State::ConstPtr& /* start_state */,
                                                    float /* timeout */) {}

  Action::ConstPtr RandomPlanner::getBestAction(const State::ConstPtr& state) const {
    std::vector<Action::ConstPtr> actions;
    model_->getActionsAtState(state, actions);
    return actions[rng_->randomInt(actions.size() - 1)];
  }

  void RandomPlanner::performPreActionProcessing(const State::ConstPtr& /* state */,
                                                 float /* timeout */) {}

  void RandomPlanner::performPostActionProcessing(const State::ConstPtr& /* state */,
                                                  const Action::ConstPtr& /* action */,
                                                  float /* timeout */) {}
  std::string RandomPlanner::getName() const {
    return std::string("Random");
  }

} /* utexas_planning */

CLASS_LOADER_REGISTER_CLASS(utexas_planning::RandomPlanner, utexas_planning::AbstractPlanner);
