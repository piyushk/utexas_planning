#!/usr/bin/env python

import argparse
import filesystem as fs
import numpy as np
import pandas as pd
from scipy import stats
import sys

parser = argparse.ArgumentParser()
parser.add_argument("data", help="The directory containing csv files or a single csv file.", type=str)
parser.add_argument("batch_headers", help="Comma separated list of constraints for batch. Ex: a=2,b=5 will split dataset into those satisfying those conditions and those that don't.", type=str)
parser.add_argument("--column", help="Column being compared for stats test. Defaults to reward.", type=str, default='reward')
parser.add_argument("--constraints", help="Comma separated list of constraints applied to all batches.", type=str)
parser.add_argument("--other-batch-headers", help="Comma separated list of constraints for the second header. This will be used instead of the rows where onal batch with slashes separating batches. Ex: a=3,b=4/a=4,b=5 will be compared to --batch-headers being true.", type=str)

# parser.add_argument("--paired", help="Run paired tests, requires number of rows in batches to be same",action="store_true")
# parser.add_argument("--p", help="p-value. Defaults to 0.05",type=float,default=0.05)
#
args = parser.parse_args()

filenames = fs.expand_path_to_filelist(args.data)
data_per_file = []
for file in filenames:
    data_per_file.append(pd.read_csv(file))
data = pd.tools.merge.concat(data_per_file, ignore_index=True)

print "Data has " + str(len(data)) + " rows."

if args.constraints is not None:
    constraints = [x.strip() for x in args.constraints.split(',')]
    for constraint in constraints:
        data = data.query(constraint)

print "Constrained data has " + str(len(data)) + " rows."

batch_header_data = data.copy()

comparison_operators = ['==' , '!=', '<>', '>', '<', '>=', '<=']
batch_headers = [x.strip() for x in args.batch_headers.split(',')]
for constraint in batch_headers:
    if not any(comparator in constraint for comparator in comparison_operators):
        constraint = constraint + "==True"
    batch_header_data = batch_header_data.query(constraint)

# from IPython import embed; embed()

if args.other_batch_headers is None:
    other_batch_headers = '!(' + args.batch_headers + ')'
    other_batch_header_data = data.drop(batch_header_data.index)
else:
    other_batch_header_data = data.copy()
    other_batch_headers = [x.strip() for x in args.other_batch_headers.split(',')]
    for constraint in other_batch_headers:
        if not any(comparator in constraint for comparator in comparison_operators):
            constraint = constraint + "==True"
        other_batch_header_data = other_batch_header_data.query(constraint)

    if not set(batch_header_data.index).isdisjoint(other_batch_header_data.index):
        print "Ruhro! It seems like there are common rows between batch_headers and other_batch_headers. Check your query"
        sys.exit(-1)

    other_batch_headers = args.other_batch_headers

print "Batch \"" + args.batch_headers + "\" has " + str(len(batch_header_data)) + " rows, with mean: ",
print np.mean(batch_header_data[args.column])
print "Batch \"" + other_batch_headers + "\" has " + str(len(other_batch_header_data)) + " rows, with mean: ",
print np.mean(other_batch_header_data[args.column])

_, p = stats.ttest_ind(batch_header_data[args.column], other_batch_header_data[args.column], equal_var=False)
print "p-value = " + str(p)
