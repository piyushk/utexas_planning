#ifndef UTEXAS_PLANNING_VI_H_
#define UTEXAS_PLANNING_VI_H_

#include <boost/shared_ptr.hpp>

#include <utexas_planning/common/params.h>
#include <utexas_planning/common/utils.h>
#include <utexas_planning/core/declarative_model.h>
#include <utexas_planning/core/abstract_planner.h>
#include <utexas_planning/planners/vi/estimator.h>

namespace utexas_planning {

  namespace vi {

    class ValueIteration {
      public:

#define PARAMS(_) \
      _(float,gamma,gamma,1.0) \
      _(float,epsilon,epsilon,1e-2) \
      _(unsigned int,max_iter,max_iter,1000) \
      _(float,max_value,max_value,std::numeric_limits<float>::max()) \
      _(float,min_value,min_value,-std::numeric_limits<float>::max()) \
      _(bool,reuse_policy_from_file,reuse_policy_from_file,false) \
      _(float,policy_computation_timeout,policy_computation_timeout,NO_TIMEOUT) \

      Params_STRUCT(PARAMS)
#undef PARAMS

        ValueIteration();
        virtual ~ValueIteration ();

        virtual void init(const boost::shared_ptr<const GenerativeModel> &model,
                          const YAML::Node params,
                          const std::string &output_directory);
        virtual void performEpisodeStartProcessing(const State &start_state, float timeout = NO_TIMEOUT);
        virtual Action getBestAction(const State &state) const;
        virtual void performPostActionProcessing(const State& state, const Action& action, float timeout = NO_TIMEOUT);
        virtual std::string getSolverName() const;
        virtual std::map<std::string, std::string> getParamsAsMap() const;

      private:

        void computePolicy(float timeout);
        void loadPolicy(const std::string& file);
        void savePolicy(const std::string& file) const;
        std::string generatePolicyFileName() const;
        std::string policy_file_location_;

        boost::shared_ptr<DeclarativeModel> model_;
        boost::shared_array<const State> states_;
        unsigned int num_states_;
        boost::shared_ptr<Estimator> value_estimator_;

        Params params_;
        bool initialized_;
        bool policy_available_;

    };

  } /* vi */

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_VI_H_ */
