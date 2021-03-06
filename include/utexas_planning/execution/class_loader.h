#ifndef UTEXAS_PLANNING_CLASS_LOADER_H_
#define UTEXAS_PLANNING_CLASS_LOADER_H_

#include <boost/algorithm/string/join.hpp>
#include <class_loader/multi_library_class_loader.h>
#include <string>
#include <utexas_planning/core/abstract_planner.h>
#include <utexas_planning/core/generative_model.h>
#include <utexas_planning/core/visualizer.h>
#include <yaml-cpp/yaml.h>

namespace utexas_planning {

  class ClassLoader {

    public:

      typedef boost::shared_ptr<ClassLoader> Ptr;

      ClassLoader();

      void addLibraries(const std::vector<std::string>& libraries);

      GenerativeModel::Ptr loadModel(const std::string& model_class,
                                     const boost::shared_ptr<RNG>& rng,
                                     const YAML::Node& params = YAML::Node(),
                                     const std::string& output_directory = "./");

      AbstractPlanner::Ptr loadPlanner(const std::string& planner_class,
                                       const GenerativeModel::ConstPtr& model,
                                       const boost::shared_ptr<RNG>& rng,
                                       const YAML::Node& params = YAML::Node(),
                                       const std::string& output_directory = "./",
                                       bool verbose = false);

      Visualizer::Ptr loadVisualizer(const std::string& visualizer_class);

    private:

      class_loader::MultiLibraryClassLoader class_loader_;

  }; /* ClassLoader */

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_CLASS_LOADER_H_ */
