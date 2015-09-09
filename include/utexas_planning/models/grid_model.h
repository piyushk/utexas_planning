#ifndef UTEXAS_PLANNING_GRID_MODEL_H_
#define UTEXAS_PLANNING_GRID_MODEL_H_

#include <boost/lexical_cast.hpp>
#include <boost/serialization/export.hpp>

#include <utexas_planning/common/constants.h>
#include <utexas_planning/common/params.h>
#include <utexas_planning/common/rng.h>
#include <utexas_planning/core/declarative_model.h>

namespace utexas_planning {

  enum GridActionType {
    UP = 0,
    DOWN = 1,
    LEFT = 2,
    RIGHT = 3,
    NUM_ACTIONS = 4
  };

  class GridModelRewardMetrics : public RewardMetrics {
    public:
      typedef boost::shared_ptr<GridModelRewardMetrics> Ptr;
      typedef boost::shared_ptr<const GridModelRewardMetrics> ConstPtr;

      bool halfway_reached;
      float halfway_reward;
      virtual std::map<std::string, std::string> asMap() const {
        std::map<std::string, std::string> metric_map;
        metric_map["halfway_reward"] = boost::lexical_cast<std::string>(halfway_reward);
        return metric_map;
      }
  };

  class GridAction : public Action {
    public:
      GridActionType type;
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

BOOST_CLASS_EXPORT_KEY(utexas_planning::GridAction);

namespace utexas_planning {

  class GridState : public State {
    public:
      int x;
      int y;

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

BOOST_CLASS_EXPORT_KEY(utexas_planning::GridState);

namespace utexas_planning {

  class GridModel : public DeclarativeModel {

    public:

#define PARAMS(_) \
      _(int,start_x,start_x,-1) \
      _(int,start_y,start_y,-1) \
      _(int,grid_size,grid_size,10) \
      _(float,goal_reward,goal_reward,0.0f) \
      _(std::string,terminal_states,terminal_states,"") \
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
      virtual RewardMetrics::Ptr getRewardMetricsAtEpisodeStart() const;
      virtual std::string getName() const;

    protected:

      bool halfway_reached_;
      std::vector<State::ConstPtr> complete_state_vector_;
      std::map<State::ConstPtr, float, State::PtrComparator> terminal_states_to_reward_map_;
      std::vector<Action::ConstPtr> default_action_list_;
      Params params_;
  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_GRID_MODEL_H_ */
