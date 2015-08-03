#include <string>
#include <stdexcept>

#include <utexas_planning/common/exceptions.h>
#include <utexas_planning/core/declarative_model.h>

namespace utexas_planning {

  DeclarativeModel::~DeclarativeModel() {}

  std::vector<State::ConstPtr> DeclarativeModel::getStateVector() const {
    throw UnimplementedFunctionException(getName(), "getStateVector()");
  }

  void DeclarativeModel::takeAction(const State::ConstPtr& state,
                                    const Action::ConstPtr& action,
                                    float& reward,
                                    const RewardMetrics::Ptr& reward_metrics,
                                    State::ConstPtr& next_state,
                                    int& depth_count,
                                    float& post_action_timeout,
                                    boost::shared_ptr<RNG> rng) const {

    // NOTE: This default implementation is definitely not optimized for every domain. However this implementation is
    // probably suitable for declarative models as this function should only be called during evaluation.

    std::vector<State::ConstPtr> next_states;
    std::vector<float> rewards;
    std::vector<float> probabilities;

    getTransitionDynamics(state, action, next_states, rewards, probabilities);
    int idx = rng->select(probabilities);

    next_state = next_states[idx];
    reward = rewards[idx];

    depth_count = 1;
    post_action_timeout = 1.0f;
  }

} /* utexas_planning */
