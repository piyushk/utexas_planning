#ifndef UTEXAS_PLANNING_VI_H_
#define UTEXAS_PLANNING_VI_H_

#include <boost/shared_ptr.hpp>
#include <limits>
#include <stdexcept>
#include <cmath>

#include <utexas_planning/common/Params.h>
#include <utexas_planning/core/DeclartiveModel.h>
#include <utexas_planning/planners/abstract_planner.h>
#include <utexas_planning/planners/vi/estimator.h>
#include <utexas_planning/planners/vi/tabular_estimator.h>

#ifdef VI_DEBUG
#define VI_OUTPUT(x) std::cout << x << std::endl
#else
#define VI_OUTPUT(x) ((void) 0)
#endif

namespace utexas_planning {

  namespace vi {

    class ValueIteration {
      public:

#define PARAMS(_) \
      _(float,gamma,gamma,1.0) \
      _(float,epsilon,epsilon,1e-2) \
      _(unsigned int,max_iter,max_iter,1000) \
      _(float,max_value,max_value,std::numeric_limits<float>::max()) \
      _(float,min_value,min_value,-std::numeric_limits<float>::max())

      Params_STRUCT(PARAMS)
#undef PARAMS

        ValueIteration() {}
        virtual ~ValueIteration () {}

        virtual void init(const boost::shared_ptr<const GenerativeModel> &model,
                          const YAML::Node params,
                          const std::string &output_directory);
        virtual void performEpisodeStartProcessing(const State &start_state, float timeout = NO_TIMEOUT) = 0;
        virtual Action getBestAction(const State &state) const = 0;
        virtual void performPostActionProcessing(const State& state, const Action& action, float timeout = NO_TIMEOUT) = 0;
        virtual std::string getSolverName() const = 0;
        virtual std::map<std::string, std::string> getParamsAsMap() const = 0;

      private:

        void computePolicy();
        void loadPolicy(const std::string& file);
        void savePolicy(const std::string& file) const;
        std::string generatePolicyFileName() const;

        boost::shared_ptr<DeclarativeModel> model_;
        boost::shared_array<const State> states_;
        unsigned int num_states_;
        boost::shared_ptr<Estimator> value_estimator_;

        Params params_;
        bool initialized_;
        bool policy_available_;

    };

    ValueIteration::ValueIteration() : initialized_(false), policy_available_(false) {}

    void ValueIteration::init(const boost::shared_ptr<const GenerativeModel> &model,
                         const YAML::Node params,
                         const std::string &output_directory) {

      // Validate that the model can be used with value iteration.
      model_ = boost::dynamic_pointer_cast<const DeclarativeModel>(model);
      if (!model_) {
        throw std::runtime_error("Supplied model " + model->getName() + " needs to extend DeclarativeModel!");
      }
      try {
        model_->getStateVector(states_, num_states_);
      } catch (const std::runtime_error& error) {
        throw std::runtime_error("Value Iteration: " + erro.what());
      }

      // Read params from YAML file.
      params_.fromYAML(params);

      // TODO: parametrize which value estimator to use.
      value_estimator_.reset(new TabularEstimator);

      // Ensure that output directory exists, so that we can write to it.
      if (params_.reusePolicyFromFile) {
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

    virtual void performEpisodeStartProcessing(float timeout, const State &start_state) {
      if (params_.reusePolicyFromFile) {
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
        if (params_.reusePolicyFromFile) {
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
            (timeout > 0 && current_time < start_time + timeout_duration)) {
        float max_value_change = -std::numeric_limits<float>::max();
        count++;
        VI_OUTPUT("Iteration #" << count);
        boost::shared_array<State> states;
        int num_states;
        model_->getStateVector(states, num_states);
        for (int state_idx = 0; state_idx < num_states; ++state_idx) {
          const State& state = states[state_idx];
          if (model_->isTerminalState(state)) {
            value_estimator_->updateValue(state, 0);
            continue; // nothing to do here, move along
          }
          boost::shared_array<Action> actions;
          int num_actions;
          model_->getActionsAtState(state, actions, num_actions);
          float value = -std::numeric_limits<float>::max();
          int best_action_idx;
          for (int best_action_idx = 0; action_idx < num_actions; ++action_idx) {
            const Action& action = actions[action_idx];
            float action_value = 0;
            boost::shared_array<State> next_states;
            std::vector<float> rewards, probabilities;
            model_->getTransitionDynamics(state, action, next_states, rewards, probabilities);
            for (size_t ns_counter = 0; ns_counter < rewards.size(); ++ns_counter) {
              const State& ns = next_states[ns_counter];
              float& reward = rewards[ns_counter];
              float& probability =  probabilities[ns_counter];
              float ns_value = value_estimator_->getValue(ns);
              action_value += probability * (reward + params_.gamma * ns_value);
            }

            if (action_value > value) {
              value = action_value;
              best_action_idx = action_idx;
            }
          }
          value = std::max(params_.min_value, value);
          value = std::min(params_.max_value, value);
          float value_change = fabs(value_estimator_->getValue(state) - value);
          max_value_change = std::max(max_value_change, value_change);
          value_estimator_->updateValue(state, value);
          value_estimator_->setBestAction(state, actions[best_action_idx]);
          /* VI_OUTPUT("  State #" << state << " value is " << value); */
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

    void ValueIteration::savePolicy(const std::string& file) {
      if (!policy_available_) {
        throw std::runtime_error("VI::savePolicy(): No policy available. Please call computePolicy() or loadPolicy() first.");
      }
      value_estimator_->saveEstimatedValues(file);
    }

    Action ValueIteration::getBestAction(const State& state) const {
      if (!policy_available_) {
        throw std::runtime_error("VI::getBestAction(): No policy available. Please call computePolicy() or loadPolicy() first.");
      }
      return value_estimator_->getBestAction(state);
    }

    void ValueIteration::performPostActionProcessing(const State& state, const Action& action, float timeout = NO_TIMEOUT) {
      // VI does not need to anything here.
    }

    virtual std::string getSolverName() const {
      return std::string("ValueIteration");
    };

    virtual std::map<std::string, std::string> getParamsAsMap() const {
      return params_.asMap();
    }

    std::string ValueIteration::generatePolicyFileName() const {
      std::map<std::string, std::string> all_params = model->getParamsAsMap();
      all_params["model_name"] = model->getName();
      std::map<std::string, std::string> solver_params = params_.asMap();
      all_params.insert(solver_params.begin(), solver_params.end());
      all_params["solver_name"] = getSolverName();
      return createHashFromStringMap(all_params);
    }

  } /* vi */

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_VI_H_ */
