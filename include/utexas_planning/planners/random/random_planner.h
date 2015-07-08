#ifndef UTEXAS_PLANNING_VI_H_
#define UTEXAS_PLANNING_VI_H_

#include <boost/shared_ptr.hpp>

#include <utexas_planning/core/abstract_planner.h>
#include <utexas_planning/common/rng.h>

namespace utexas_planning {

  namespace random {

    class RandomPlanner : public AbstractPlanner {

      public:

        typedef boost::shared_ptr<RandomPlanner> Ptr;
        typedef boost::shared_ptr<const RandomPlanner> ConstPtr;

        virtual ~RandomPlanner ();

        virtual void init(const GenerativeModel::ConstPtr& model,
                          const YAML::Node& params,
                          const std::string& output_directory,
                          const boost::shared_ptr<RNG>& rng);
        virtual void performEpisodeStartProcessing(const State::ConstPtr& start_state = State::ConstPtr(),
                                                   float timeout = NO_TIMEOUT);
        virtual Action::ConstPtr getBestAction(const State::ConstPtr& state) const;
        virtual void performPostActionProcessing(const State::ConstPtr& state,
                                                 const Action::ConstPtr& action,
                                                 float timeout = NO_TIMEOUT);

      private:

        boost::shared_ptr<RNG> rng_;
        GenerativeModel::ConstPtr model_;
    };

  } /* random */

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_VI_H_ */
