#ifndef UTEXAS_PLANNING_UTILS_H_
#define UTEXAS_PLANNING_UTILS_H_

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <map>
#include <string>

#include <utexas_planning/common/farmhash.h>

namespace utexas_planning {

    std::string createHashFromStringMap(const std::map<std::string, std::string> &string_map) {
      typedef std::pair<std::string, std::string> SSPair;
      std::string hash_str = "";
      BOOST_FOREACH(const SSPair& pair, string_map) {
        hash_str += pair.second;
      }
      uint64_t hash = Fingerprint64(hash_str.c_str(), hash_str.size());
      return boost::lexical_cast<std::string>(hash);
    }

};

#endif /* end of include guard: UTEXAS_PLANNING_UTILS_H_ */
