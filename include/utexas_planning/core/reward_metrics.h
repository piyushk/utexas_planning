#ifndef UTEXAS_PLANNING_REWARD_METRICS_H_
#define UTEXAS_PLANNING_REWARD_METRICS_H_

namespace utexas_planning {

  class RewardMetrics {

    public:

      typedef boost::shared_ptr<RewardMetrics> Ptr;
      typedef boost::shared_ptr<const RewardMetrics> ConstPtr;

      virtual std::map<std::string, std::string> asMap() const {
        return std::map<std::string, std::string>();
      }
  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_REWARD_METRICS_H_ */
