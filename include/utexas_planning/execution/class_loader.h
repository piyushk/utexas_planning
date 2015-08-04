#ifndef UTEXAS_PLANNING_CLASS_LOADER_H_
#define UTEXAS_PLANNING_CLASS_LOADER_H_

#include <boost/algorithm/string/join.hpp>
#include <class_loader/multi_library_class_loader.h>
#include <string>
#include <utexas_planning/core/abstract_planner.h>
#include <utexas_planning/core/generative_model.h>
#include <yaml-cpp/yaml.h>

namespace utexas_planning {

  class ClassLoader {

    public:

      static ClassLoader& getInstance() {
        static ClassLoader instance;
        return instance;
      }

      void addLibraries(const std::vector<std::string>& libraries);

      GenerativeModel::ConstPtr loadModel(const std::string& model_class,
                                          const YAML::Node& params = YAML::Node(),
                                          const std::string& output_directory = "./");

      AbstractPlanner::Ptr loadPlanner(const std::string& planner_class,
                                       const GenerativeModel::ConstPtr& model,
                                       const boost::shared_ptr<RNG>& rng,
                                       const YAML::Node& params = YAML::Node(),
                                       const std::string& output_directory = "./");

    private:

      // Ensure that only a single instance is created here.
      ClassLoader();
      ClassLoader(ClassLoader const&);
      void operator=(ClassLoader const&);

      class_loader::MultiLibraryClassLoader class_loader_;

  }; /* ClassLoader */

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_CLASS_LOADER_H_ */
