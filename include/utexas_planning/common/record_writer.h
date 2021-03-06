#ifndef UTEXAS_PLANNING_RECORD_WRITER
#define UTEXAS_PLANNING_RECORD_WRITER

#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <boost/algorithm/string/join.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/copy.hpp>

namespace utexas_planning {

  inline bool writeRecordsAsCSV(const std::string &filename,
                                const std::vector<std::map<std::string, std::string> > &records) {

    // Get the superset of keys for all records.
    std::set<std::string> all_keys;
    typedef std::map<std::string, std::string> StringMap;
    BOOST_FOREACH(const StringMap& record, records) {
      // TODO std::inserter may be an inefficient way of copying the keys to the all_keys set
      boost::copy(record | boost::adaptors::map_keys, std::inserter(all_keys, all_keys.begin()));
    }

    // Create the file and write the header
    std::ofstream ofs(filename.c_str());
    if (!ofs.is_open()) {
      return false;
    }
    ofs << boost::algorithm::join(all_keys, ",") << std::endl;

    // Go through each record, check if a key is there, otherwise insert a 0.
    BOOST_FOREACH(const StringMap& record, records) {
      std::vector<std::string> vals;
      BOOST_FOREACH(const std::string& key, all_keys) {
        StringMap::const_iterator value_iter = record.find(key);
        if (value_iter == record.end()) {
          vals.push_back("0");
        } else {
          try {
            boost::lexical_cast<float>(value_iter->second);
            vals.push_back(value_iter->second);
          } catch(const boost::bad_lexical_cast &) {
            // Non numerical/boolean type. Enclose within quotes
            vals.push_back("\"" + value_iter->second + "\"");
          }
        }
      }
      ofs << boost::algorithm::join(vals, ",") << std::endl;
    }

    ofs.close();
  }

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_RECORD_WRITER */
