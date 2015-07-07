#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <utexas_planning/common/exceptions.h>
#include <utexas_planning/planners/vi/tabular_estimator.h>
#include <utexas_planning/planners/vi/vi.h>

#ifdef VI_DEBUG
#define VI_OUTPUT(x) std::cout << x << std::endl
#else
#define VI_OUTPUT(x) ((void) 0)
#endif

namespace utexas_planning {

  namespace vi {

    ValueIteration::ValueIteration() : initialized_(false), policy_available_(false) {}
    ValueIteration::~ValueIteration() {}

    void ValueIteration::init(const GenerativeModel::ConstPtr &model,
                         const YAML::Node &params,
                         const std::string &output_directory) {

      // Validate that the model can be used with value iteration.
      model_ = boost::dynamic_pointer_cast<const DeclarativeModel>(model);
      if (!model_) {
        throw IncorrectUsageException("ValueIteration: Supplied model " + model->getName() +
                                      " needs to extend DeclarativeModel for use with VI!");
      }
      try {
        states_ = model_->getStateVector();
      } catch (const UnimplementedFunctionException& error) {
        throw IncorrectUsageException(std::string("ValueIteration requires the following to be implemented: ") +
                                      std::string(error.what()));
      }

      // Read params from YAML file.
      params_.fromYaml(params);

      // TODO: parametrize which value estimator to use.
      value_estimator_.reset(new TabularEstimator);

      // Ensure that output directory exists, so that we can write to it.
      if (params_.reuse_policy_from_file) {
        if (!boost::filesystem::is_directory(output_directory) &&
            !boost::filesystem::create_directory(output_directory)) {
          std::cerr << "Output directory (" << output_directory << ") does not exist, but unable to create it. " <<
            "The situation can still be valid if another process created the directory at the same time." << std::endl;

        }

        // Cache the policy file name for convenience
        boost::filesystem::path dir (output_directory);
        boost::filesystem::path file (generatePolicyFileName());
        boost::filesystem::path full_path = dir / file;
        policy_file_location_ = full_path.string();
      }

      initialized_ = true;
    }

    void ValueIteration::performEpisodeStartProcessing(const State::ConstPtr& /* start_state */, float timeout) {
      if (params_.reuse_policy_from_file) {
        if (boost::filesystem::is_regular_file(policy_file_location_)) {
          loadPolicy(policy_file_location_);
        }
      }
      if (!policy_available_) {
        if (timeout > 0 && params_.policy_computation_timeout > 0) {
          timeout = std::min(timeout, params_.policy_computation_timeout);
        } else if (params_.policy_computation_timeout > 0) {
          timeout = params_.policy_computation_timeout;
        }
        computePolicy(timeout);
        if (params_.reuse_policy_from_file) {
          savePolicy(policy_file_location_);
        }
      }
    }

    void ValueIteration::computePolicy(float timeout) {

      bool change = true;
      size_t count = 0;

      boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
      boost::posix_time::ptime current_time = start_time;
      boost::posix_time::time_duration timeout_duration = boost::posix_time::milliseconds(timeout * 1000);

      while((change) &&
            (count < params_.max_iter) &&
            (timeout < 0 || current_time < start_time + timeout_duration)) {
        float max_value_change = -std::numeric_limits<float>::max();
        count++;
        VI_OUTPUT("Iteration #" << count);
        BOOST_FOREACH(const State::ConstPtr& state, states_) {
          if (model_->isTerminalState(state)) {
            value_estimator_->setValueAndBestAction(state, 0);
            continue; // nothing to do here, move along
          }
          std::vector<boost::shared_ptr<const Action> > actions;
          model_->getActionsAtState(state, actions);
          float value = -std::numeric_limits<float>::max();
          Action::ConstPtr best_action;
          BOOST_FOREACH(const Action::ConstPtr& action, actions) {
            float action_value = 0;
            std::vector<State::ConstPtr> next_states;
            std::vector<float> rewards, probabilities;
            model_->getTransitionDynamics(state, action, next_states, rewards, probabilities);
            for (size_t ns_counter = 0; ns_counter < rewards.size(); ++ns_counter) {
              float& reward = rewards[ns_counter];
              float& probability =  probabilities[ns_counter];
              float ns_value = value_estimator_->getValue(next_states[ns_counter]);
              action_value += probability * (reward + params_.gamma * ns_value);
            }

            if (action_value > value) {
              value = action_value;
              best_action = action;
            }
          }
          value = std::max(params_.min_value, value);
          value = std::min(params_.max_value, value);
          float current_value = value_estimator_->getValue(state);
          float value_change = fabs(current_value - value);
          max_value_change = std::max(max_value_change, value_change);
          value_estimator_->setValueAndBestAction(state, value, best_action);
          /* VI_OUTPUT("  State #" << *state << " value is " << value); */
        }
        VI_OUTPUT("  max change = " << max_value_change);
        change = max_value_change > params_.epsilon;
        current_time = boost::posix_time::microsec_clock::local_time();
      }
      policy_available_ = true;
    }

    void ValueIteration::loadPolicy(const std::string& file) {
      value_estimator_->loadEstimatedValues(file);
      policy_available_ = true;
    }

    void ValueIteration::savePolicy(const std::string& file) const {
      value_estimator_->saveEstimatedValues(file);
    }

    Action::ConstPtr ValueIteration::getBestAction(const State::ConstPtr& state) const {
      Action::ConstPtr best_action = value_estimator_->getBestAction(state);
      if (!best_action) {
        throw IncorrectUsageException("ValueIteration::getBestAction(): provided state has not been seen before! "
                                      "Either this state is not a valid state, or performEpisodeStartProcessing() "
                                      "was not called prior to requesting the best action.");
      }
      return best_action;
    }

    void ValueIteration::performPostActionProcessing(const State::ConstPtr& /* state */,
                                                     const Action::ConstPtr& /* action */,
                                                     float /* timeout */) {
      // VI does not need to anything here.
    }

    std::map<std::string, std::string> ValueIteration::getParamsAsMap() const {
      return params_.asMap();
    }

    std::string ValueIteration::generatePolicyFileName() const {
      std::map<std::string, std::string> all_params = model_->getParamsAsMap();
      all_params["model_name"] = model_->getName();
      std::map<std::string, std::string> planner_params = params_.asMap();
      all_params.insert(planner_params.begin(), planner_params.end());
      all_params["planner_name"] = getName();
      return createHashFromStringMap(all_params);
    }

  } /* vi */

} /* utexas_planning */
