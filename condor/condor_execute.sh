#!/bin/bash

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <experiment-file> <num-trials>" >&2
  exit 1
fi

if [ ! -f $1 ]; then
  echo "$1 should be a valid YAML experiment file."
  exit 1
fi

CURRENT_DIR=`pwd`
CONDOR_FILE=`mktemp /tmp/XXXXXXXXXX.condor`
cp /u/piyushk/utexas_planning/utexas_planning/condor/template.condor $CONDOR_FILE
echo "Arguments = --data-directory $CURRENT_DIR --experiment-file $1 --seed \$(Process)" >> $CONDOR_FILE
echo "Queue $2" >> $CONDOR_FILE

condor_submit $CONDOR_FILE
rm -f $CONDOR_FILE
