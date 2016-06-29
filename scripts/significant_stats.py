#!/usr/bin/env python

import argparse
import filesystem as fs
import numpy as np
import pandas as pd
from scipy import stats
import sys

COMPARISON_OPERATORS = ['==' , '!=', '<>', '>', '<', '>=', '<=']

def get_constraints_list_from_string(constraints_string):
    constraints_list = [x.strip() for x in constraints_string.split(',')]
    return [x+"==True" if not any(comparator in x for comparator in COMPARISON_OPERATORS) else x for x in constraints_list]

def get_constrained_data(source, constraints_str):
    if constraints_str is None:
        return source
    data = source.copy()
    constraints = get_constraints_list_from_string(constraints_str)
    for constraint in constraints:
        data = data.query(constraint)
    return data

parser = argparse.ArgumentParser()
parser.add_argument("data", help="The directory containing csv files or a single csv file.", type=str)
parser.add_argument("batch_headers", help="Comma separated list of constraints for batch. Ex: a=2,b=5 will split dataset into those satisfying those conditions and those that don't.", type=str)
parser.add_argument("--column", help="Column being compared for stats test. Defaults to \"reward\"", type=str, default='reward')
parser.add_argument("--constraints", help="Comma separated list of constraints applied to all batches.", type=str)
parser.add_argument("--other-batch-headers", help="Comma separated list of constraints for the optional second header.", type=str)

# parser.add_argument("--paired", help="Run paired tests, requires number of rows in batches to be same",action="store_true")
# parser.add_argument("--p", help="p-value. Defaults to 0.05",type=float,default=0.05)

args = parser.parse_args()

filenames = fs.expand_path_to_filelist(args.data)
data_per_file = []
for file in filenames:
    data_per_file.append(pd.read_csv(file))
data = pd.tools.merge.concat(data_per_file, ignore_index=True)

print "Dataset has " + str(len(data)) + " rows."

data = get_constrained_data(data, args.constraints)
print "Constrained dataset has " + str(len(data)) + " rows."

batch_data = get_constrained_data(data, args.batch_headers)
batch_headers = args.batch_headers

if args.other_batch_headers is None:
    other_batch_headers = '!(' + args.batch_headers + ')'
    other_batch_data = data.drop(batch_data.index)
else:
    other_batch_headers = args.other_batch_headers
    other_batch_data = get_constrained_data(data, args.other_batch_headers)

    if not set(batch_data.index).isdisjoint(other_batch_data.index):
        print >> sys.stderr, "The dataset has common rows between batch_headers and other_batch_headers; check your query."
        sys.exit(-1)

print "Batch \"" + args.batch_headers + "\" has " + str(len(batch_data)) + " rows, with mean: ",
print np.mean(batch_data[args.column])
print "Batch \"" + other_batch_headers + "\" has " + str(len(other_batch_data)) + " rows, with mean: ",
print np.mean(other_batch_data[args.column])

_, p = stats.ttest_ind(batch_data[args.column], other_batch_data[args.column], equal_var=False)
print "p-value = " + str(p)
