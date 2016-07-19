#ifndef UTEXAS_PLANNING_UTILS_H_
#define UTEXAS_PLANNING_UTILS_H_

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <map>
#include <string>

#include <utexas_planning/common/exceptions.h>
#include <utexas_planning/common/farmhash.h>

namespace utexas_planning {

  inline std::string createHashFromStringMap(const std::map<std::string, std::string>& string_map) {
    typedef std::pair<std::string, std::string> SSPair;
    std::string hash_str = "";
    BOOST_FOREACH(const SSPair& pair, string_map) {
      hash_str += pair.second;
    }
    uint64_t hash = util::Fingerprint64(hash_str.c_str(), hash_str.size());
    return boost::lexical_cast<std::string>(hash);
  }

  inline std::vector<std::string> getLibrariesFromEnvironment(
      const std::string env = std::string("UTEXAS_PLANNING_LIBRARIES")) {

    char* libraries_as_char;
    libraries_as_char = getenv(env.c_str());
    if (libraries_as_char == NULL) {
      throw IncorrectUsageException(env + std::string(" environment variable not set!"));
    }
    std::string libraries_as_str(libraries_as_char);
    std::vector<std::string> libraries, libraries_filtered;
    boost::split(libraries, libraries_as_str, boost::is_any_of(",;:"));

    // Convert any directories to actual libraries in the directory.
    for (std::vector<std::string>::iterator lib_it = libraries.begin(); lib_it != libraries.end(); ++lib_it) {
      boost::filesystem::path dir_path(*lib_it);
      if (boost::filesystem::is_directory(dir_path)) {
        boost::filesystem::recursive_directory_iterator dir_it(dir_path);
        boost::filesystem::recursive_directory_iterator end_it;
        while (dir_it != end_it) {
          if (boost::filesystem::is_regular_file(*dir_it) && dir_it->path().extension() == ".so")  {
            libraries_filtered.push_back(boost::filesystem::canonical(dir_it->path()).string());
          }
          ++dir_it;
        }
      } else {
        libraries_filtered.push_back(*lib_it);
      }
    }

    return libraries_filtered;
  }

}

#endif /* end of include guard: UTEXAS_PLANNING_UTILS_H_ */
