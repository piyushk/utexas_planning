#ifndef UTEXAS_PLANNING_VI_H_
#define UTEXAS_PLANNING_VI_H_

#include <boost/shared_ptr.hpp>
#include <limits>
#include <stdexcept>
#include <cmath>

#include <utexas_planning/common/Params.h>
#include <utexas_planning/core/DeclartiveModel.h>
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

        ValueIteration (boost::shared_ptr<DeclarativeModel> model,
                        boost::shared_ptr<Estimator> value_estimator = boost::shared_ptr<Estimator>(),
                        Params params = Params());

        virtual ~ValueIteration () {}

        void computePolicy();
        void loadPolicy(const std::string& file);
        void savePolicy(const std::string& file);
        Action getBestAction(const State& state) const;

      private:

        boost::shared_ptr<DeclarativeModel> model_;
        boost::shared_ptr<Estimator> value_estimator_;

        Params params_;
        bool policy_available_;

    };

    ValueIteration::ValueIteration(boost::shared_ptr<DeclarativeModel> model,
                                   boost::shared_ptr<Estimator> value_estimator,
                                   Params params) :
      model_(model),
      value_estimator_(value_estimator),
      params_(params),
      policy_available_(false) {
        if (!value_estimator_) {
          value_estimator_.reset(new TabularEstimator(model));
        }
      }

    void ValueIteration::computePolicy() {

      bool change = true;
      size_t count = 0;
      while(change && count < params_.max_iter) {
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

    const Action& ValueIteration::getBestAction(const State& state) const {
      if (!policy_available_) {
        throw std::runtime_error("VI::getBestAction(): No policy available. Please call computePolicy() or loadPolicy() first.");
      }
      return value_estimator_->getBestAction(state);
    }

  } /* vi */

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_VI_H_ */
