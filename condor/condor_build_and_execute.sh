#!/bin/bash

set -e

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
  echo "Usage: $0 <experiment-file> [<submit-node>]" >&2
  exit 1
fi


