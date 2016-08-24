#ifndef UTEXAS_PLANNING_LIGHT_WORLD_MODEL_H_
#define UTEXAS_PLANNING_LIGHT_WORLD_MODEL_H_

#include <boost/serialization/export.hpp>

#include <utexas_planning/common/constants.h>
#include <utexas_planning/common/params.h>
#include <utexas_planning/common/rng.h>
#include <utexas_planning/core/declarative_model.h>

namespace utexas_planning {

  enum SimpleGuidanceActionType {
    UP = 0,
    DOWN = 1,
    LEFT = 2,
    RIGHT = 3,
    NOOP = 4,
    NUM_ACTIONS = 5
  };

  class SimpleGuidanceAction : public Action {
    public:
      SimpleGuidanceActionType type;
      bool operator<(const Action& other_base) const;
      void serialize(std::ostream& stream) const;

    private:
      friend class boost::serialization::access;
      template <typename Archive> void serialize(Archive& ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Action);
        ar & BOOST_SERIALIZATION_NVP(type);
      }
  };

} /* utexas_planning */

BOOST_CLASS_EXPORT_KEY(utexas_planning::SimpleGuidanceAction);

namespace utexas_planning {

  class SimpleGuidanceState : public State {
    public:
      int x;
      int y;
      SimpleGuidanceAction prev_action;

      bool operator<(const State& other_base) const;
      void serialize(std::ostream& stream) const;
      std::map<std::string, std::string> asMap() const;

    private:

      State::Ptr cloneImpl() const;

      friend class boost::serialization::access;
      template <typename Archive> void serialize(Archive& ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(State);
        ar & BOOST_SERIALIZATION_NVP(x);
        ar & BOOST_SERIALIZATION_NVP(y);
      }
  };

} /* utexas_planning */

BOOST_CLASS_EXPORT_KEY(utexas_planning::SimpleGuidanceState);

namespace utexas_planning {

  class SimpleGuidanceModel : public DeclarativeModel {

    public:

#define PARAMS(_) \
      _(int,grid_size,grid_size,40) \
      _(int,num_roundabouts,num_roundabouts,80) \
      _(float,nondeterminism,nondeterminism,0.1f) \
      _(float,initial_planning_time,initial_planning_time,NO_TIMEOUT) \
      _(float,per_step_planning_time,per_step_planning_time,NO_TIMEOUT) \

      Params_STRUCT(PARAMS)
#undef PARAMS

      virtual void init(const YAML::Node& params,
                        const std::string& output_directory,
                        const boost::shared_ptr<RNG>& rng);

      virtual bool isTerminalState(const State::ConstPtr& state_base) const;

      virtual void getActionsAtState(const State::ConstPtr& state,
                             std::vector<Action::ConstPtr>& actions) const;

      virtual std::vector<State::ConstPtr> getStateVector() const;

      virtual void getTransitionDynamics(const State::ConstPtr& state_base,
                                 const Action::ConstPtr& action_base,
                                 std::vector<State::ConstPtr>& next_states,
                                 std::vector<float>& rewards,
                                 std::vector<float>& probabilities) const;

      virtual void takeAction(const State::ConstPtr& state,
                              const Action::ConstPtr& action,
                              float& reward,
                              const RewardMetrics::Ptr& reward_metrics,
                              State::ConstPtr& next_state,
                              int& depth_count,
                              float& post_action_timeout,
                              boost::shared_ptr<RNG> rng) const;

      virtual State::ConstPtr getStartState(long seed);

      virtual float getInitialTimeout() const;
      virtual std::map<std::string, std::string> getParamsAsMap() const;
      virtual std::string getName() const;

    protected:

      inline int getStateIndex(const SimpleGuidanceState& state) const {
        int idx =
          (state.prev_action.type) + (state.y * 4) + (state.x * params_.grid_size * 4);
        return idx;
      }

      std::vector<State::ConstPtr> complete_state_vector_;
      std::vector<std::pair<int, int> > roundabouts_;
      Params params_;

      boost::shared_ptr<SimpleGuidanceAction> up_action;
      boost::shared_ptr<SimpleGuidanceAction> down_action;
      boost::shared_ptr<SimpleGuidanceAction> left_action;
      boost::shared_ptr<SimpleGuidanceAction> right_action;
      boost::shared_ptr<SimpleGuidanceAction> noop_action;
  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_LIGHT_WORLD_MODEL_H_ */
