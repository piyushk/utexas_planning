#!/bin/bash

if [ "$#" -ne 2 ] && [ "$#" -ne 3 ]; then
  echo "Usage: $0 <directory> <num-trials> [<trial-length>]" >&2
  exit 1
fi

if [ ! -d $1 ]; then
  echo "$1 should be a valid directory."
  exit 1
fi

shopt -s nullglob
for f in $1/*yaml
do
  basename=$(basename "$f" ".yaml")
  mkdir $basename && cd $basename && /u/piyushk/utexas_planning/utexas_planning/condor/condor_execute.sh "$f" $2 $3 && cd ../
done
