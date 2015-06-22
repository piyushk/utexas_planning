#ifndef UTEXAS_PLANNING_VI_ESTIMATOR_H_
#define UTEXAS_PLANNING_VI_ESTIMATOR_H_

#include <string>

#include <utexas_planning/common/Action.h>
#include <utexas_planning/common/State.h>

class VIEstimator {
  public:
    VIEstimator () {}
    virtual ~VIEstimator () {}

    virtual float getValue(const State &state) = 0;
    virtual void updateValue(const State &state, float value) = 0;
    virtual Action getBestAction(const State &state) = 0;
    virtual void setBestAction(const State &state, const Action& action) = 0;

    virtual void saveEstimatedValues(const std::string& file) = 0;
    virtual void loadEstimatedValues(const std::string& file) = 0;
};


#endif /* end of include guard: UTEXAS_PLANNING_VI_ESTIMATOR_H_ */
