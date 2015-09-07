#ifndef UTEXAS_PLANNING_LIGHT_WORLD_MODEL_H_
#define UTEXAS_PLANNING_LIGHT_WORLD_MODEL_H_

#include <boost/serialization/export.hpp>

#include <utexas_planning/common/constants.h>
#include <utexas_planning/common/params.h>
#include <utexas_planning/common/rng.h>
#include <utexas_planning/core/declarative_model.h>

namespace utexas_planning {

  enum LightWorldActionType {
    UP = 0,
    DOWN = 1,
    LEFT = 2,
    RIGHT = 3,
    PICKUP = 4,
    UNLOCK = 5,
    NUM_ACTIONS = 6
  };

  class LightWorldAction : public Action {
    public:
      LightWorldActionType type;
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

BOOST_CLASS_EXPORT_KEY(utexas_planning::LightWorldAction);

namespace utexas_planning {

  class LightWorldState : public State {
    public:
      int x;
      int y;
      bool key_picked_up;
      bool goal_unlocked;
      int unlock_attempts_left;

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

BOOST_CLASS_EXPORT_KEY(utexas_planning::LightWorldState);

namespace utexas_planning {

  class LightWorldModel : public DeclarativeModel {

    public:

#define PARAMS(_) \
      _(int,start_x,start_x,0) \
      _(int,start_y,start_y,0) \
      _(int,key_x,key_x,1) \
      _(int,key_y,key_y,3) \
      _(int,lock_x,lock_x,4) \
      _(int,lock_y,lock_y,0) \
      _(int,goal_x,goal_x,4) \
      _(int,goal_y,goal_y,3) \
      _(int,initial_unlock_attempts,initial_unlock_attempts,5) \
      _(int,grid_size,grid_size,5) \
      _(float,incorrect_pickup_reward,incorrect_pickup_reward,-1.0f) \
      _(float,incorrect_unlock_reward,incorrect_unlock_reward,-1.0f) \
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

      virtual State::ConstPtr getStartState(long seed) const;

      virtual float getInitialTimeout() const;
      virtual std::map<std::string, std::string> getParamsAsMap() const;
      virtual std::string getName() const;

    protected:

      inline int getStateIndex(const LightWorldState& state) const {
        int idx =
          state.unlock_attempts_left +
          (state.goal_unlocked * (params_.initial_unlock_attempts + 1)) +
          (state.key_picked_up * 2 * (params_.initial_unlock_attempts + 1)) +
          (state.y * 2 * 2 * (params_.initial_unlock_attempts + 1)) +
          (state.x * (params_.grid_size) * 2 * 2 * (params_.initial_unlock_attempts + 1));
        return idx;
      }

      std::vector<State::ConstPtr> complete_state_vector_;
      Params params_;

      boost::shared_ptr<LightWorldAction> up_action;
      boost::shared_ptr<LightWorldAction> down_action;
      boost::shared_ptr<LightWorldAction> left_action;
      boost::shared_ptr<LightWorldAction> right_action;
      boost::shared_ptr<LightWorldAction> pickup_action;
      boost::shared_ptr<LightWorldAction> unlock_action;
  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_LIGHT_WORLD_MODEL_H_ */
