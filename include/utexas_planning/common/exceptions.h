#ifndef UTEXAS_PLANNING_EXCEPTIONS_H_
#define UTEXAS_PLANNING_EXCEPTIONS_H_

#include <stdexcept>

namespace utexas_planning {

  class UnimplementedFunctionException : public std::runtime_error {
    public:
      UnimplementedFunctionException(const std::string& name, const std::string& function_name) :
        std::runtime_error(name + " does not implement " + function_name + ", but one of the solvers " +
                           "being used requires this function to be implemented.") {}
  };

  class DowncastException : public std::runtime_error {
    public:
      DowncastException(const std::string& from_class, const std::string& to_class) :
        std::runtime_error("Expected downcast from base class " + from_class + " to class " + to_class + " failed!") {}
  };

  class IncorrectUsageException : public std::runtime_error {
    public:
      IncorrectUsageException(const std::string& incorrect_usage) : std::runtime_error(incorrect_usage) {}
  };

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_EXCEPTIONS_H_ */
