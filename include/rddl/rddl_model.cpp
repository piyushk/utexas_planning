#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <class_loader/class_loader.h>

#include "rddl_model.h"

namespace utexas_planning {

  bool RddlAction::operator<(const Action& other_base) const {
    try {
      const RddlAction& other = dynamic_cast<const RddlAction&>(other_base);
      return state < other.state;
    } catch(const std::bad_cast& exp) {
      throw DowncastException("Action", "RddlAction");
    }
  }

  bool RddlAction::operator==(const Action& other_base) const {
    try {
      const RddlAction& other = dynamic_cast<const RddlAction&>(other_base);
      return ((!(state < other.state)) &&
              (!(other.state < state)));
    } catch(const std::bad_cast& exp) {
      throw DowncastException("Action", "RddlAction");
    }
  }

  void RddlAction::serialize(std::ostream& stream) const {
    stream << state.getName();
  }

} /* utexas_planning */

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::RddlAction);

namespace utexas_planning {

  bool RddlState::operator<(const State& other_base) const {
    try {
      const RddlState& other = dynamic_cast<const RddlState&>(other_base);
      if (state < other.state) return true;
      if (other.state < state) return false;

      if (remaining_steps < other.remaining_steps) return true;
      if (other.remaining_steps < remaining_steps) return false;

      return false;
    } catch(const std::bad_cast& exp) {
      throw DowncastException("State", "RddlState");
    }
  }

  void RddlState::serialize(std::ostream& stream) const {
    state.print(stream);
  }

  State::Ptr RddlState::cloneImpl() const {
    boost::shared_ptr<RddlState> clone(new RddlState);
    clone.state.reset(new rddl::State(state));
    return clone;
  }

  std::map<std::string, std::string> RddlState::asMap() const {
    std::map<std::string, std::string> state_map;
    for (int i = 0; i < state.state.size(); ++i) {
      state_map["s" + boost::lexical_cast<std::string>(i)] = boost::lexical_cast<std::string(state.state[i]);
    }
    return state_map;
  }

} /* utexas_planning */

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::RddlState);

namespace utexas_planning {

  void RddlModel::init(const YAML::Node& params,
                         const std::string& /* output_directory */,
                         const boost::shared_ptr<RNG>& rng) {

    params_.fromYaml(params);

    // TODO update rddl_domain, and rddl_problem with the actual file locations.

    rddl::RDDLParser parser;
    task_ = parser.parse(params_.rddl_domain, prams_.rddl_problem);
    rddl::Instantiator instantiator(task_);
    instantiator.instantiate();
    rddl::Preprocessor preprocessor(task_);
    preprocessor.preprocess();

    // TODO generate default action list and start state.
  }

  bool RddlModel::isTerminalState(const State::ConstPtr& state_base) const {
    boost::shared_ptr<const RddlState> state = boost::dynamic_pointer_cast<const RddlState>(state_base);
    if (!state) {
      throw DowncastException("State", "RddlState");
    }
    return state->remaining_steps == 0;
  }

  void RddlModel::getActionsAtState(const State::ConstPtr& state,
                                    std::vector<Action::ConstPtr>& actions) const {

    // TODO Return default action list barring any that fail state-action-constraints.
  }

  void RddlModel::takeAction(const State::ConstPtr& state,
                             const Action::ConstPtr& action,
                             float& reward,
                             const RewardMetrics::Ptr& /* reward_metrics */,
                             State::ConstPtr& next_state,
                             int& depth_count,
                             float& post_action_timeout,
                             boost::shared_ptr<RNG> rng) const {
    // TODO not sure how to supply rng to evaluatable.
    post_action_timeout = params_.per_step_planning_time;
    depth_count = 1;

  }

  State::ConstPtr RddlModel::getStartState(long seed) const {
    return start_state_;
  }

  float RddlModel::getInitialTimeout() const {
    return params_.initial_planning_time;
  }

  std::string RddlModel::getName() const {
    return std::string("Grid3D");
  }

} /* utexas_planning */

CLASS_LOADER_REGISTER_CLASS(utexas_planning::RddlModel, utexas_planning::GenerativeModel);
