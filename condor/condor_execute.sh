#!/bin/bash

if [ "$#" -ne 2 ] && [ "$#" -ne 3 ]; then
  echo "Usage: $0 <experiment-file> <num-trials> [<trial-length>]" >&2
  exit 1
fi

if [ ! -f $1 ]; then
  echo "$1 should be a valid YAML experiment file."
  exit 1
fi

CURRENT_DIR=`pwd`
CONDOR_FILE=`mktemp /tmp/XXXXXXXXXX.condor`
cp /u/piyushk/utexas_planning/utexas_planning/condor/template.condor $CONDOR_FILE

if [ "$#" -eq 2 ]; then
  ARGUMENTS="Arguments = --data-directory $CURRENT_DIR --experiment-file $1 --seed \$(Process)"
else
  ARGUMENTS="Arguments = --data-directory $CURRENT_DIR --experiment-file $1 --max-trial-length $3 --seed \$(Process)"
fi
echo "$ARGUMENTS" >> $CONDOR_FILE
echo "Queue $2" >> $CONDOR_FILE

mkdir -p logs
cat $CONDOR_FILE
rm -f $CONDOR_FILE
