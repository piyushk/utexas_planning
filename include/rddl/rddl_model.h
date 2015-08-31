#ifndef UTEXAS_PLANNING_RDDL_MODEL_H_
#define UTEXAS_PLANNING_RDDL_MODEL_H_

#include <boost/serialization/export.hpp>

#include <utexas_planning/common/constants.h>
#include <utexas_planning/common/exceptions.h>
#include <utexas_planning/common/params.h>
#include <utexas_planning/common/rng.h>
#include <utexas_planning/core/declarative_model.h>

#include "states.h"
#include "planning_task.h"

namespace utexas_planning {

  class RddlAction : public Action {
    public:
      boost::shared_ptr<rddl::ActionState> state;

      bool operator<(const Action& other_base) const;
      bool operator==(const Action& other_base) const;
      void serialize(std::ostream& stream) const;

    private:
      friend class boost::serialization::access;
      template <typename Archive> void serialize(Archive& ar, const unsigned int version) {
        throw UnimplementedFunctionException("RddlAction","serialize(Archive& ar)");
      }
  };

} /* utexas_planning */

BOOST_CLASS_EXPORT_KEY(utexas_planning::RddlAction);

namespace utexas_planning {

  class RddlState : public State {
    public:
      boost::shared_ptr<rddl::State> state;
      int remaining_steps;
      rddl::PlanningTask* task;

      bool operator<(const State& other_base) const;
      void serialize(std::ostream& stream) const;
      std::map<std::string, std::string> asMap() const;

    private:

      State::Ptr cloneImpl() const;

      friend class boost::serialization::access;
      template <typename Archive> void serialize(Archive& ar, const unsigned int version) {
        throw UnimplementedFunctionException("RddlAction","serialize(Archive& ar)");
      }
  };

} /* utexas_planning */

BOOST_CLASS_EXPORT_KEY(utexas_planning::RddlState);

namespace utexas_planning {

  class RddlModel : public GenerativeModel {

    public:

#define PARAMS(_) \
      _(std::string,rddl_domain,rddl_domain,"triangle_tireworld") \
      _(std::string,rddl_problem,rddl_problem,"5") \
      _(float,initial_planning_time,initial_planning_time,NO_TIMEOUT) \
      _(float,per_step_planning_time,per_step_planning_time,NO_TIMEOUT) \

      Params_STRUCT(PARAMS)
#undef PARAMS

      RddlModel();

      virtual void init(const YAML::Node& params,
                        const std::string& output_directory,
                        const boost::shared_ptr<RNG>& rng);

      virtual bool isTerminalState(const State::ConstPtr& state_base) const;

      virtual void getActionsAtState(const State::ConstPtr& state_base,
                                     std::vector<Action::ConstPtr>& actions) const;

      virtual void takeAction(const State::ConstPtr& state_base,
                              const Action::ConstPtr& action_base,
                              float& reward,
                              const RewardMetrics::Ptr& reward_metrics,
                              State::ConstPtr& next_state,
                              int& depth_count,
                              float& post_action_timeout,
                              boost::shared_ptr<RNG> rng) const;

      virtual State::ConstPtr getStartState(long seed) const;

      virtual float getInitialTimeout() const;
      virtual std::string getName() const;

    private:

      std::vector<State::ConstPtr> complete_state_vector_;
      std::vector<Action::ConstPtr> default_action_list_;

      Params params_;
      rddl::PlanningTask* task_;
      boost::shared_ptr<RddlState> start_state_;

  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_RDDL_MODEL_H_ */
