#include <utexas_planning/core/generative_model.h>

namespace utexas_planning {

  GenerativeModel::~GenerativeModel() {}

  std::map<std::string, std::string> GenerativeModel::getParamsAsMap() const {
    return std::map<std::string, std::string>();
  }

} /* utexas_planning */
