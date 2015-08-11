#ifndef UTEXAS_PLANNING_GRID_MODEL_H_
#define UTEXAS_PLANNING_GRID_MODEL_H_

#include <boost/serialization/export.hpp>

#include <utexas_planning/common/params.h>
#include <utexas_planning/common/rng.h>
#include <utexas_planning/core/declarative_model.h>

namespace utexas_planning {

  class GridAction3D : public Action {
    public:
      int xdiff;
      int ydiff;
      int zdiff;
      bool operator<(const Action& other_base) const;
      bool operator==(const Action& other_base) const;
      void serialize(std::ostream& stream) const;


    private:
      friend class boost::serialization::access;
      template <typename Archive> void serialize(Archive& ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Action);
        ar & BOOST_SERIALIZATION_NVP(xdiff);
        ar & BOOST_SERIALIZATION_NVP(ydiff);
        ar & BOOST_SERIALIZATION_NVP(zdiff);
      }
  };

} /* utexas_planning */

BOOST_CLASS_EXPORT_KEY(utexas_planning::GridAction3D);

namespace utexas_planning {

  class GridState3D : public State {
    public:
      int x;
      int y;
      int z;

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
        ar & BOOST_SERIALIZATION_NVP(z);
      }
  };

} /* utexas_planning */

BOOST_CLASS_EXPORT_KEY(utexas_planning::GridState3D);

namespace utexas_planning {

  class GridModel3D : public DeclarativeModel {

    public:

#define PARAMS(_) \
      _(int,start_x,start_x,-1) \
      _(int,start_y,start_y,-1) \
      _(int,start_z,start_z,-1) \
      _(int,grid_size,grid_size,10) \
      _(int,num_actions,num_actions,6) \
      _(float,non_determinism,non_determinism,0.1f) \

      Params_STRUCT(PARAMS)
#undef PARAMS

      virtual void init(const YAML::Node& params,
                        const std::string& output_directory,
                        const boost::shared_ptr<RNG>& rng);

      bool isTerminalState(const State::ConstPtr& state_base) const;

      void getActionsAtState(const State::ConstPtr& state,
                             std::vector<Action::ConstPtr>& actions) const;

      std::vector<State::ConstPtr> getStateVector() const;

      void getTransitionDynamics(const State::ConstPtr& state_base,
                                 const Action::ConstPtr& action_base,
                                 std::vector<State::ConstPtr>& next_states,
                                 std::vector<float>& rewards,
                                 std::vector<float>& probabilities) const;

      State::ConstPtr getStartState(long seed) const;

    private:

      std::vector<State::ConstPtr> complete_state_vector_;
      std::vector<Action::ConstPtr> default_action_list_;
      Params params_;
  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_GRID_MODEL_H_ */
