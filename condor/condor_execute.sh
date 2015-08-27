#!/bin/bash

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <experiment-file> <num-trials>" >&2
  exit 1
fi

if [ ! -f $1 ]; then
  echo "$1 should be a valid YAML experiment file."
  exit 1
fi

echo "Running quick test on local machine for 10 seconds to ensure that experiment file is readable."
/u/piyushk/utexas_planning/install/bin/evaluator --exp $1 --verbose --max-trial-depth 1 &
BG_PROC_PID=$!

sleep 10
kill $BG_PROC_ID

# If the kill command fails, it means the process already terminated, and there is a problem reading the experiment file.
if [ "$?" -ne 0 ]; then
  echo "Test evaluation run failed within 10 seconds. Please fix and re-run." >&2
  exit 1
fi

set -e

rm -f result.0

CURRENT_DIR=`pwd`
CONDOR_FILE=`mktemp /tmp/XXXXXXXXXX.condor`
cp /u/piyushk/utexas_planning/utexas_planning/condor/template.condor $CONDOR_FILE
sed -i "s/@ARGS@/--data-directory $CURRENT_DIR --experiment-file $1/" $CONDOR_FILE
sed -i "s/@TRIALS@/$2/" $CONDOR_FILE
cat $CONDOR_FILE
rm -f $CONDOR_FILE

# condor_submit $CONDOR_FILE
