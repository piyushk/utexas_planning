#include <utexas_planning/core/generative_model.h>

namespace utexas_planning {

  GenerativeModel::~GenerativeModel() {}

  void GenerativeModel::takeAction(const State::ConstPtr& state,
                                   const Action::ConstPtr& action,
                                   float& reward,
                                   State::ConstPtr& next_state,
                                   boost::shared_ptr<RNG> rng) const {
    int unused_depth_count;
    takeAction(state, action, reward, next_state, unused_depth_count, rng);
  };

  void GenerativeModel::takeAction(const State::ConstPtr& state,
                                   const Action::ConstPtr& action,
                                   float& reward,
                                   State::ConstPtr& next_state,
                                   int& depth_count,
                                   boost::shared_ptr<RNG> rng) const {
    float unused_timeout;
    RewardMetrics::Ptr unused_reward_metrics;
    takeAction(state, action, reward, unused_reward_metrics, next_state, depth_count, unused_timeout, rng);
  }

  std::map<std::string, std::string> GenerativeModel::getParamsAsMap() const {
    return std::map<std::string, std::string>();
  }

  std::string GenerativeModel::getName() const {
    return typeid(*this).name();
  }

  RewardMetrics::Ptr GenerativeModel::getRewardMetricsAtEpisodeStart() {
    return RewardMetrics::Ptr(new RewardMetrics);
  }

} /* utexas_planning */
