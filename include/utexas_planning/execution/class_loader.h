#ifndef UTEXAS_PLANNING_CLASS_LOADER_H_
#define UTEXAS_PLANNING_CLASS_LOADER_H_

#include <boost/algorithm/string/join.hpp>
#include <class_loader/multi_library_class_loader.h>

namespace utexas_planning {

  class ClassLoader {

    public:

      static ClassLoader& getInstance() {
        static ClassLoader instance;
        return instance;
      }

      inline void addLibraries(const std::vector<std::string>& libraries) {
        BOOST_FOREACH(const std::string& library, libraries) {
          class_loader_.loadLibrary(library);
        }
      }

      inline GenerativeModel::ConstPtr loadModel(const std::string& model_class,
                                                 const YAML::Node& params = YAML::Node(),
                                                 const std::string& output_directory = "/tmp"} {
        std::vector<std::string> classes = class_loader_.getAvailableClasses<GenerativeModel>();
        BOOST_FOREACH(const std::string& class_name, classes) {
          if (class_name == model_class) {
            GenerativeModel::Ptr model = loader.createInstance<GenerativeModel const>(class_name);
            model->init(params, output_directory);
            return model;
          }
        }
        std::string all_available_classes = "[" + boost::algorithm::join(classes, ", ") + "]";
        throw ResourceNotFoundException("ClassLoader: Unable to load requested class " + model_class +
                                        ". Available classes are " + all_available_classes);
      }

      inline AbstractPlanner::Ptr loadPlanner(const std::string& planner_class,
                                              const GenerativeModel::ConstPtr& model,
                                              const boost::shared_ptr<RNG>& rng,
                                              const YAML::Node& params = YAML::Node(),
                                              const std::string& output_directory = "/tmp"} {
        std::vector<std::string> classes = class_loader_.getAvailableClasses<AbstractPlanner>();
        BOOST_FOREACH(const std::string& class_name, classes) {
          if (class_name == model_class) {
            AbstractPlanner::Ptr model = loader.createInstance<AbstractPlanner const>(class_name);
            model->init(model, params, output_directory, rng);
            return model;
          }
        }
        std::string all_available_classes = "[" + boost::algorithm::join(classes, ", ") + "]";
        throw ResourceNotFoundException("ClassLoader: Unable to load requested class " + model_class +
                                        ". Available classes are " + all_available_classes);
      }

    private:

      // Ensure that only a single instance is created here.
      ClassLoader() {}
      ClassLoader(ClassLoader const&) = delete;
      void operator=(ClassLoader const&) = delete;

      class_loader::MultiLibraryClassLoader class_loader_;

  }; /* ClassLoader */

} /* utexas_planning */

#endif /* end of include guard: UTEXAS_PLANNING_CLASS_LOADER_H_ */
