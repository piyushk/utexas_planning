#!/bin/bash

# This command assumes all necessary shared object libraries have been loaded into
export UTEXAS_PLANNING_LIBRARIES=/u/piyushk/utexas_planning/install/lib/
export RDDL_DOMAIN_DIRECTORIES=/u/piyushk/utexas_planning/utexas_planning/benchmarks/rddl_prefix
export LD_LIBRARY_PATH=/u/piyushk/utexas_planning/install/lib:/u/piyushk/utexas_planning/install/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}

~/utexas_planning/install/bin/evaluator "$@"
