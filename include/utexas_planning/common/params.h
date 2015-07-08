#ifndef UTEXAS_PLANNING_PARAMS_H_
#define UTEXAS_PLANNING_PARAMS_H_

#include <boost/lexical_cast.hpp>
#include <map>
#include <string>
#include <yaml-cpp/yaml.h>

#define SET_FROM_YAML(type) \
  inline void setFromYaml(const YAML::Node& node, const char *key, type& val) { \
    if (node[key]) \
      val = node[key].as<type>(); \
  }

SET_FROM_YAML(bool)
SET_FROM_YAML(int)
SET_FROM_YAML(unsigned int)
SET_FROM_YAML(double)
SET_FROM_YAML(float)
SET_FROM_YAML(std::string)

#define PARAM_DECL(type,var,key,val) type var;
#define PARAM_INIT(type,var,key,val) var = val;
#define PARAM_SET(type,var,key,val) setFromYaml(node,#key,var);
#define PARAM_OUT(type,var,key,val) os << #var << ": " << p.var << " ";
#define PARAM_MAP(type,var,key,val) stringMap[#var] = boost::lexical_cast<std::string>(var);

#define Params_STRUCT(params) \
  struct Params {\
    params(PARAM_DECL) \
    \
    Params() { \
      params(PARAM_INIT) \
    } \
    \
    void fromYaml(const YAML::Node& node) { \
      (void)node; /* to remove any compiler warnings if params is empty */ \
      params(PARAM_SET) \
    } \
    std::map<std::string, std::string> asMap() const { \
      std::map<std::string, std::string> stringMap; \
      params(PARAM_MAP) \
      return stringMap; \
    } \
    friend std::ostream& operator<<(std::ostream& os, const Params& p) { \
      params(PARAM_OUT) \
      return os; \
    } \
  };

#endif /* end of include guard: UTEXAS_PLANNING_PARAMS_H_ */
