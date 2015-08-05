#include <class_loader/class_loader.h>
#include <utexas_planning/planners/random/random_planner.h>

namespace utexas_planning {

  namespace random {

    RandomPlanner::~RandomPlanner() {}

    void RandomPlanner::init(const GenerativeModel::ConstPtr& model,
                             const YAML::Node& /* params */,
                             const std::string& /* output_directory */,
                             const boost::shared_ptr<RNG>& rng) {
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

    void RandomPlanner::performPostActionProcessing(const State::ConstPtr& /* state */,
                                                    const Action::ConstPtr& /* action */,
                                                    float /* timeout */) {}

  } /* random */

} /* utexas_planning */

CLASS_LOADER_REGISTER_CLASS(utexas_planning::random::RandomPlanner, utexas_planning::AbstractPlanner);
