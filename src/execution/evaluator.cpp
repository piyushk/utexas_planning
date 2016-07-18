#include<fstream>
#include<cstdlib>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>

#include <utexas_planning/common/constants.h>
#include <utexas_planning/common/record_writer.h>
#include <utexas_planning/execution/class_loader.h>
#include <utexas_planning/execution/evaluation.h>

using namespace utexas_planning;

YAML::Node experiment_;
std::string experiment_file_;
std::string visualizer_class_ = "utexas_planning::NoVisualization";
std::string data_directory_ = ".";      // runtime directory.
int seed_ = 0;
int num_instances_ = 1;
bool verbose_ = false;
bool manual_ = false;
bool post_action_processing_ = false;
int max_trial_depth_ = NO_MAX_DEPTH;
float max_trial_time_ = NO_TIMEOUT;

ClassLoader::Ptr loader_;
Visualizer::Ptr visualizer_;

int processOptions(int argc, char** argv) {

  std::string mcts_params_file, methods_file;

  /** Define and parse the program options
  */
  namespace po = boost::program_options;
  po::options_description desc("Options");
  desc.add_options()
    ("experiment-file", po::value<std::string>(&experiment_file_)->required(),
     "JSON file containing all the necessary information about this experiment.")
    ("data-directory", po::value<std::string>(&data_directory_), "Data directory (defaults to runtime directory).")
    ("visualizer_", po::value<std::string>(&visualizer_class_), "Dynamically loaded class name for visualizing the domain. Empty means no visuzalization.")
    ("seed", po::value<int>(&seed_), "Random seed (process number on condor)")
    ("num-instances", po::value<int>(&num_instances_), "Number of Instances")
    ("verbose", "Increased verbosity in planner and trial output.")
    ("manual", "Use manual action selection instead of planner.")
    ("post-action-processing", "Planners plan using previous state and selected action instead of planning using the current state.")
    ("max-trial-depth", po::value<int>(&max_trial_depth_), "Maximum number of actions that should be taken inside MDP in case a terminal state is not reached.")
    ("max-trial-time", po::value<float>(&max_trial_time_), "Maximum time each trial should run for in case a terminal state is not reached.");

  po::variables_map vm;

  try {
    po::store(po::command_line_parser(argc, argv).options(desc)
              /* .positional(positionalOptions).allow_unregistered().run(),  */
              .allow_unregistered().run(),
              vm); // throws on error

    po::notify(vm); // throws on error, so do after help in case
    // there are any problems
  } catch(boost::program_options::required_option& e) {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cout << desc << std::endl;
    return -1;
  } catch(boost::program_options::error& e) {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cout << desc << std::endl;
    return -1;
  }

  if (vm.count("verbose")) {
    verbose_ = true;
  }

  if (vm.count("manual")) {
    manual_ = true;
  }

  if (vm.count("post-action-processing")) {
    post_action_processing_ = true;
  }

  /* Read in methods */
  std::cout << "Experiment File: " << experiment_file_ << std::endl;
  // TODO catch an exception here.
  experiment_ = YAML::LoadFile(experiment_file_);

  /* Create the output directory */
  if (!boost::filesystem::is_directory(data_directory_) && !boost::filesystem::create_directory(data_directory_)) {
    std::cerr << "Unable to create directory for storing intermediate output and results: " << data_directory_;
    return -1;
  }

  /* Visualization is only supported for one model at a time. */
  // YAML::Node models_yaml = experiment_["models"];
  // if (models_yaml.size() != 1 && visualizer_class_ != "utexas_planning::NoVisualization") {
  //   std::cerr << "Visualizing more than one model is not supported through the default evaluator." << std::endl;
  //   return -1;
  // }
  return 0;
}

void runExperiment() {

  boost::shared_ptr<RNG> rng(new RNG(seed_));

  YAML::Node models_yaml = experiment_["models"];
  YAML::Node planners_yaml = experiment_["planners"];
  std::vector<std::map<std::string, std::string> > records;
  for (unsigned model_idx = 0; model_idx < models_yaml.size(); ++model_idx) {
    boost::shared_ptr<RNG> model_rng(new RNG(rng->randomInt()));
    std::string model_name = models_yaml[model_idx]["name"].as<std::string>();
    GenerativeModel::ConstPtr model = loader_->loadModel(model_name, model_rng, models_yaml[model_idx], data_directory_);
    model->initializeVisualizer(visualizer_);
    for (unsigned planner_idx = 0; planner_idx < planners_yaml.size(); ++planner_idx) {
      boost::shared_ptr<RNG> planner_rng(new RNG(rng->randomInt()));
      std::string planner_name = planners_yaml[planner_idx]["name"].as<std::string>();
      AbstractPlanner::Ptr planner = loader_->loadPlanner(planner_name,
                                                         model,
                                                         planner_rng,
                                                         planners_yaml[planner_idx],
                                                         data_directory_,
                                                         verbose_);
      records.push_back(runSingleTrial(model,
                                       planner,
                                       visualizer_,
                                       data_directory_,
                                       seed_,
                                       max_trial_depth_,
                                       max_trial_time_,
                                       post_action_processing_,
                                       verbose_,
                                       manual_));
    }
  }

  // Running trials
  writeRecordsAsCSV(data_directory_ + "/result." + boost::lexical_cast<std::string>(seed_), records);

}

int main(int argc, char** argv) {

  int ret = processOptions(argc, argv);
  if (ret != 0) {
    return ret;
  }

  // Load all libraries_as_char from environment variable.
  loader_.reset(new ClassLoader);

  char* libraries_as_char;
  libraries_as_char = getenv("UTEXAS_PLANNING_LIBRARIES");
  if (libraries_as_char == NULL) {
    std::cerr << "UTEXAS_PLANNING_LIBRARIES environment variable not set!" << std::endl;
    return -1;
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

  loader_->addLibraries(libraries_filtered);

  visualizer_ = loader_->loadVisualizer(visualizer_class_);
  visualizer_->init(argc, argv);

  boost::thread experiment_thread(runExperiment);

  visualizer_->exec();

  experiment_thread.join();

  return 0;
}
