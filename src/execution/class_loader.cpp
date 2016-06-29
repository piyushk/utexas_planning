#include <boost/algorithm/string/join.hpp>
#include <boost/foreach.hpp>

#include <utexas_planning/common/exceptions.h>
#include <utexas_planning/execution/class_loader.h>

namespace utexas_planning {

  ClassLoader::ClassLoader() : class_loader_(false) {}

  void ClassLoader::addLibraries(const std::vector<std::string>& libraries) {
    BOOST_FOREACH(const std::string& library, libraries) {
      class_loader_.loadLibrary(library);
    }
  }

  GenerativeModel::ConstPtr ClassLoader::loadModel(const std::string& model_class,
                                                   const boost::shared_ptr<RNG>& rng,
                                                   const YAML::Node& params,
                                                   const std::string& output_directory) {
    std::vector<std::string> classes = class_loader_.getAvailableClasses<GenerativeModel>();
    BOOST_FOREACH(const std::string& class_name, classes) {
      if (class_name == model_class) {
        GenerativeModel::Ptr model = class_loader_.createInstance<GenerativeModel>(class_name);
        model->init(params, output_directory, rng);
        return model;
      }
    }
    std::string all_available_classes = "[" + boost::algorithm::join(classes, ", ") + "]";
    throw ResourceNotFoundException("ClassLoader: Unable to load requested class " + model_class +
                                    ". Available classes are " + all_available_classes);
  }

  AbstractPlanner::Ptr ClassLoader::loadPlanner(const std::string& planner_class,
                                                const GenerativeModel::ConstPtr& model,
                                                const boost::shared_ptr<RNG>& rng,
                                                const YAML::Node& params,
                                                const std::string& output_directory,
                                                bool verbose) {
    std::vector<std::string> classes = class_loader_.getAvailableClasses<AbstractPlanner>();
    BOOST_FOREACH(const std::string& class_name, classes) {
      if (class_name == planner_class) {
        AbstractPlanner::Ptr planner = class_loader_.createInstance<AbstractPlanner>(class_name);
        planner->init(model, params, output_directory, rng, verbose);
        return planner;
      }
    }
    std::string all_available_classes = "[" + boost::algorithm::join(classes, ", ") + "]";
    throw ResourceNotFoundException("ClassLoader: Unable to load requested class " + planner_class +
                                    ". Available classes are " + all_available_classes);
  }

  Visualizer::Ptr ClassLoader::loadVisualizer(const std::string& visualizer_class) {
    std::vector<std::string> classes = class_loader_.getAvailableClasses<Visualizer>();
    BOOST_FOREACH(const std::string& class_name, classes) {
      if (class_name == visualizer_class) {
        return class_loader_.createInstance<Visualizer>(class_name);
      }
    }
    std::string all_available_classes = "[" + boost::algorithm::join(classes, ", ") + "]";
    throw ResourceNotFoundException("ClassLoader: Unable to load requested class " + visualizer_class +
                                    ". Available classes are " + all_available_classes);
    
  }

} /* utexas_planning */
