#include <string>
#include <iostream>
#include <fstream>
#include <cstdlib>

#include "rddl_parser.h"
#include "instantiator.h"
#include "preprocessor.h"
#include "task_analyzer.h"
#include "planning_task.h"

#include "utils/timer.h"

using namespace std;

void printUsage() {
    cout << "Usage: ./rddl-parser <rddlDomain> <rddlProblem> <targetDir>" <<
    endl << endl;
}

int main(int argc, char** argv) {
    Timer totalTime;
    if (argc < 4) {
        printUsage();
        return 1;
    }

    // Read non-optionals
    string domainFile = string(argv[1]);
    string problemFile = string(argv[2]);
    string targetDir = string(argv[3]);

    double seed = time(NULL);

    // Read optinals
    for (unsigned int i = 4; i < argc; ++i) {
        string nextOption = string(argv[i]);
        if (nextOption == "-s") {
            seed = atoi(string(argv[++i]).c_str());
        } else {
            assert(false);
        }
    }

    srand(seed);

    // Parse, instantiate and preprocess domain and problem
    Timer t;
    cout << "parsing..." << endl;
    RDDLParser parser;
    PlanningTask* task = parser.parse(domainFile, problemFile);
    cout << "...finished (" << t << ")." << endl;

    std::cout << task->nonFluents.size() << std::endl;
    std::cout << task->stateFluents.size() << std::endl;
    std::cout << task->actionFluents.size() << std::endl;
    std::cout << task->SACs.size() << std::endl;
    std::cout << task->actionPreconds.size() << std::endl;

    t.reset();
    cout << "instantiating..." << endl;
    Instantiator instantiator(task);
    instantiator.instantiate();
    cout << "...finished (" << t << ")." << endl;

    std::cout << task->nonFluents.size() << std::endl;
    std::cout << task->stateFluents.size() << std::endl;
    std::cout << task->actionFluents.size() << std::endl;
    std::cout << task->SACs.size() << std::endl;
    std::cout << task->actionPreconds.size() << std::endl;

    /* task->print(cout); */

    t.reset();
    cout << "preprocessing..." << endl;
    Preprocessor preprocessor(task);
    preprocessor.preprocess();
    cout << "...finished (" << t << ")." << endl;

    std::cout << task->nonFluents.size() << std::endl;
    std::cout << task->stateFluents.size() << std::endl;
    std::cout << task->actionFluents.size() << std::endl;
    std::cout << task->SACs.size() << std::endl;
    std::cout << task->actionPreconds.size() << std::endl;

    /* task->print(cout); */

    return 0;
}
