#include<cstdlib>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <class_loader/class_loader.h>

#include "rddl_model.h"

#include "evaluatables.h"
#include "rddl_parser.h"
#include "instantiator.h"
#include "preprocessor.h"
#include "utils/math_utils.h"

namespace utexas_planning {

  bool RddlAction::operator<(const Action& other_base) const {
    try {
      const RddlAction& other = dynamic_cast<const RddlAction&>(other_base);
      return *state < *(other.state);
    } catch(const std::bad_cast& exp) {
      throw DowncastException("Action", "RddlAction");
    }
  }

  bool RddlAction::operator==(const Action& other_base) const {
    try {
      const RddlAction& other = dynamic_cast<const RddlAction&>(other_base);
      return ((!(*state < *(other.state))) &&
              (!(*(other.state) < *state)));
    } catch(const std::bad_cast& exp) {
      throw DowncastException("Action", "RddlAction");
    }
  }

  void RddlAction::serialize(std::ostream& stream) const {
    stream << state->getName();
  }

} /* utexas_planning */

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::RddlAction);

namespace utexas_planning {

  bool RddlState::operator<(const State& other_base) const {
    try {
      const RddlState& other = dynamic_cast<const RddlState&>(other_base);
      rddl::State::StateSort sorter;
      if (sorter(*state,*(other.state))) return true;
      if (sorter(*(other.state),*state)) return false;

      if (remaining_steps < other.remaining_steps) return true;
      if (other.remaining_steps < remaining_steps) return false;

      return false;
    } catch(const std::bad_cast& exp) {
      throw DowncastException("State", "RddlState");
    }
  }

  void RddlState::serialize(std::ostream& stream) const {
    //state->print(stream);
    for (int i = 0; i < state->state.size(); ++i) {
      const double& state_member = state->state[i];
      if (rddl::MathUtils::doubleIsEqual(state_member, 1.0)) {
        stream << task->CPFs[i]->name;
      } else if (rddl::MathUtils::doubleIsEqual(state_member, 0.0)) {
        stream << "~" << task->CPFs[i]->name;
      } else {
        stream << task->CPFs[i]->name << "=" << state_member;
      }
      stream << " ";
    }
    stream << "(Remaining steps: " << remaining_steps << ")";
  }

  State::Ptr RddlState::cloneImpl() const {
    boost::shared_ptr<RddlState> clone(new RddlState);
    clone->state.reset(new rddl::State(*state));
    clone->remaining_steps = remaining_steps;
    clone->task = task;
    return clone;
  }

  std::map<std::string, std::string> RddlState::asMap() const {
    std::map<std::string, std::string> state_map;
    for (int i = 0; i < state->state.size(); ++i) {
      state_map["s" + boost::lexical_cast<std::string>(i)] = boost::lexical_cast<std::string>(state->state[i]);
    }
    return state_map;
  }

} /* utexas_planning */

BOOST_CLASS_EXPORT_IMPLEMENT(utexas_planning::RddlState);

namespace utexas_planning {

  RddlModel::RddlModel() : task_(NULL) {}

  void RddlModel::init(const YAML::Node& params,
                       const std::string& /* output_directory */,
                       const boost::shared_ptr<RNG>& rng) {

    params_.fromYaml(params);

    if (params_.rddl_problem.empty()) {
      int problem_idx = rng->randomInt(1,10);
      params_.rddl_problem =
        params_.rddl_domain + "_inst_mdp__" + boost::lexical_cast<std::string>(problem_idx) + ".rddl_prefix";
    } else {
      params_.rddl_problem =
        params_.rddl_domain + "_inst_mdp__" + params_.rddl_problem + ".rddl_prefix";
    }
    params_.rddl_domain = params_.rddl_domain + "_mdp.rddl_prefix";

    char* directories_as_char;
    directories_as_char = getenv("RDDL_DOMAIN_DIRECTORIES");
    if (directories_as_char == NULL) {
      throw IncorrectUsageException("RDDL_DOMAIN_DIRECTORIES environment variable needs to be set to use RddlModel");
    }
    std::string directories_as_str(directories_as_char);
    std::vector<std::string> directories;
    boost::split(directories, directories_as_str, boost::is_any_of(",;:"));

    bool rddl_files_found = false;
    BOOST_FOREACH(const std::string& directory, directories) {
      boost::filesystem::path dir_path(directory);
      boost::filesystem::path domain_file(params_.rddl_domain);
      boost::filesystem::path problem_file(params_.rddl_problem);
      boost::filesystem::path full_domain_path = dir_path / domain_file;
      boost::filesystem::path full_problem_path = dir_path / problem_file;
      if (boost::filesystem::exists(full_domain_path) && boost::filesystem::exists(full_problem_path)) {
        params_.rddl_domain = full_domain_path.string();
        params_.rddl_problem = full_problem_path.string();
        rddl_files_found = true;
      }
    }

    if (!rddl_files_found) {
      throw IncorrectUsageException("Unable to located RDDL description files for " + params_.rddl_domain +
                                    " and " + params_.rddl_problem + ". Searched directories specified in " +
                                    "environment variable RDDL_DOMAIN_DIRECTORIES [" +
                                    boost::algorithm::join(directories, ", ") + "].");
    }

    std::cout << "RDDL Domain File: " << params_.rddl_domain << std::endl;
    std::cout << "RDDL Problem File: " << params_.rddl_problem << std::endl;

    rddl::RDDLParser parser;
    task_ = parser.parse(params_.rddl_domain, params_.rddl_problem);
    rddl::Instantiator instantiator(task_);
    instantiator.instantiate();
    rddl::Preprocessor preprocessor(task_);
    preprocessor.preprocess();

    start_state_.reset(new RddlState);
    start_state_->state.reset(new rddl::State(task_->CPFs));
    start_state_->remaining_steps = task_->horizon;
    start_state_->task = task_;

    default_action_list_.resize(task_->actionStates.size());
    for (int idx = 0; idx < task_->actionStates.size(); ++idx) {
      rddl::ActionState& action_state = task_->actionStates[idx];
      boost::shared_ptr<RddlAction> action(new RddlAction);
      action->state.reset(new rddl::ActionState(action_state));
      default_action_list_[idx] = action;
    }

    // std::cout << default_action_list_.size() << std::endl;
    // std::cout << task_->stateFluents.size() << std::endl;
    // std::cout << task_->CPFs.size() << std::endl;

  }

  bool RddlModel::isTerminalState(const State::ConstPtr& state_base) const {
    boost::shared_ptr<const RddlState> state = boost::dynamic_pointer_cast<const RddlState>(state_base);
    if (!state) {
      throw DowncastException("State", "RddlState");
    }
    return state->remaining_steps == 0;
  }

  void RddlModel::getActionsAtState(const State::ConstPtr& state_base,
                                    std::vector<Action::ConstPtr>& actions) const {

    boost::shared_ptr<const RddlState> state = boost::dynamic_pointer_cast<const RddlState>(state_base);
    if (!state) {
      throw DowncastException("State", "RddlState");
    }

    actions.resize(default_action_list_.size());
    int valid_actions_counter = 0;

    for (int idx = 0; idx < task_->actionStates.size(); ++idx) {
      boost::shared_ptr<const RddlAction> default_action =
        boost::dynamic_pointer_cast<const RddlAction>(default_action_list_[idx]);
      const rddl::ActionState& action = *(default_action->state);
      bool action_valid = true;
      for (unsigned int precond_idx = 0; precond_idx < action.relevantSACs.size(); ++precond_idx) {
        double res = 0.0;
        action.relevantSACs[precond_idx]->formula->evaluate(res, *(state->state), action);
        if (rddl::MathUtils::doubleIsEqual(res, 0.0)) {
          action_valid = false;
          break;
        }
      }
      if (action_valid) {
        actions[valid_actions_counter] = default_action_list_[idx];
        ++valid_actions_counter;
      }
    }

    // Remove the noop action if there are more than 1 valid actions.
    // if (valid_actions_counter > 1) {
    //   actions.erase(actions.begin());
    //   actions.resize(valid_actions_counter - 1);
    // } else {
      actions.resize(valid_actions_counter);
    /* } */

  }

  void RddlModel::takeAction(const State::ConstPtr& state_base,
                             const Action::ConstPtr& action_base,
                             float& reward,
                             const RewardMetrics::Ptr& /* reward_metrics */,
                             State::ConstPtr& next_state,
                             int& depth_count,
                             float& post_action_timeout,
                             boost::shared_ptr<RNG> rng) const {

    // rddl uses srand. There's probably a better way to do this, but let's do this for now.
    srand(rng->randomUInt());

    boost::shared_ptr<const RddlState> state = boost::dynamic_pointer_cast<const RddlState>(state_base);
    if (!state) {
      throw DowncastException("State", "RddlState");
    }

    boost::shared_ptr<const RddlAction> action = boost::dynamic_pointer_cast<const RddlAction>(action_base);
    if (!action) {
      throw DowncastException("Action", "RddlAction");
    }

    if (state->remaining_steps == 0) {
      throw IncorrectUsageException("RddlModel: takeAction called on a terminal state, which is not allowed");
    }

    boost::shared_ptr<RddlState> next_state_mutable(new RddlState);
    next_state_mutable->state.reset(new rddl::State(task_->CPFs.size()));
    for(unsigned int i = 0; i < task_->CPFs.size(); ++i) {
      task_->CPFs[i]->formula->evaluate((*(next_state_mutable->state))[i], *(state->state), *(action->state));
    }
    next_state_mutable->remaining_steps = state->remaining_steps - 1;
    next_state_mutable->task = task_;
    next_state = next_state_mutable; // copy over to const container.

    double reward_as_double;
    task_->rewardCPF->formula->evaluate(reward_as_double, *(state->state), *(action->state));
    reward = reward_as_double; // implicit conversion to float.

    post_action_timeout = params_.per_step_planning_time;
    depth_count = 1;
  }

  State::ConstPtr RddlModel::getStartState(long seed) {
    return start_state_;
  }

  float RddlModel::getInitialTimeout() const {
    return params_.initial_planning_time;
  }

  std::string RddlModel::getName() const {
    std::string name = "RDDL - uninstantiated";
    if (task_ != NULL) {
      // TODO: perhaps clean this name up a bit?
      name = "RDDL - " + task_->name;
    }
    return name;
  }

} /* utexas_planning */

CLASS_LOADER_REGISTER_CLASS(utexas_planning::RddlModel, utexas_planning::GenerativeModel);
