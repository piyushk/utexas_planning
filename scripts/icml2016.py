#!/usr/bin/env python

import argparse
import filesystem as fs
import numpy as np
import pandas as pd
from scipy import stats

parser = argparse.ArgumentParser()
parser.add_argument("data", help="The directory containing csv files or a single csv file.", type=str)
args = parser.parse_args()

filenames = fs.expand_path_to_filelist(args.data)
data_per_file = []
for file in filenames:
    data_per_file.append(pd.read_csv(file))
data = pd.tools.merge.concat(data_per_file, ignore_index=True)

print "Data has the following models: " + str(pd.unique(data.model_name.ravel()))

table_latex_data = {}
table_latex_sig_mc = {}
table_latex_sig_gamma = {}

num_models = 0
for model_name in sorted(pd.unique(data.model_name.ravel())):
    num_models += 1

    # print model_name
    model_data = data[(data.model_name==model_name) & (data.action_selection_strategy=='uniform')]
    monte_carlo_data = model_data[(model_data.backup_strategy == 'backup_lambda_q') & (model_data.backup_lambda_value > 0.99)]['reward']
    max_mcts_lambda_mean = -1e20
    mcts_lambda_mean = -1e20
    for lambda_val in np.arange(0,1.05,0.1):
        max_mcts_lambda_data_temp = model_data[(model_data.backup_strategy == 'backup_lambda_q') & (model_data.backup_lambda_value > lambda_val - 0.01) & (model_data.backup_lambda_value < lambda_val + 0.01)]['reward']
        if np.mean(max_mcts_lambda_data_temp) > max_mcts_lambda_mean:
            max_mcts_lambda_mean = np.mean(max_mcts_lambda_data_temp)
            max_mcts_lambda_data = max_mcts_lambda_data_temp
            max_mcts_lambda_val = lambda_val

        mcts_lambda_data_temp = model_data[(model_data.backup_strategy == 'backup_lambda_sarsa') & (model_data.backup_lambda_value > lambda_val - 0.01) & (model_data.backup_lambda_value < lambda_val + 0.01)]['reward']
        if np.mean(mcts_lambda_data_temp) > mcts_lambda_mean:
            mcts_lambda_mean = np.mean(mcts_lambda_data_temp)
            mcts_lambda_data = mcts_lambda_data_temp
            mcts_lambda_val = lambda_val

    max_mcts_gamma_data = model_data[(model_data.backup_strategy == 'backup_gamma_q')]['reward']
    mcts_gamma_data = model_data[(model_data.backup_strategy == 'backup_gamma_sarsa')]['reward']

    # print "  MC: " + str(len(monte_carlo_data))
    # print "  MaxMCTSL: " + str(len(max_mcts_lambda_data))
    # print "  MCTSL: " + str(len(mcts_lambda_data))
    # print "  MaxMCTSG: " + str(len(max_mcts_gamma_data))
    # print "  MCTSG: " + str(len(mcts_gamma_data))

    table_latex_data[model_name] = [
        np.mean(monte_carlo_data),
        np.mean(max_mcts_lambda_data),
        max_mcts_lambda_val,
        np.mean(mcts_lambda_data),
        mcts_lambda_val,
        np.mean(max_mcts_gamma_data),
        np.mean(mcts_gamma_data)
    ]

    table_latex_sig_mc[model_name] = []
    table_latex_sig_mc[model_name].append(False)
    _, p = stats.ttest_ind(max_mcts_lambda_data, monte_carlo_data, equal_var=False)
    p_sig = p < 0.05 and np.mean(max_mcts_lambda_data) > np.mean(monte_carlo_data)
    table_latex_sig_mc[model_name].append(p_sig)
    table_latex_sig_mc[model_name].append(False)
    _, p = stats.ttest_ind(mcts_lambda_data, monte_carlo_data, equal_var=False)
    p_sig = p < 0.05 and np.mean(mcts_lambda_data) > np.mean(monte_carlo_data)
    table_latex_sig_mc[model_name].append(p_sig)
    table_latex_sig_mc[model_name].append(False)
    _, p = stats.ttest_ind(max_mcts_gamma_data, monte_carlo_data, equal_var=False)
    p_sig = p < 0.05 and np.mean(max_mcts_gamma_data) > np.mean(monte_carlo_data)
    table_latex_sig_mc[model_name].append(p_sig)
    _, p = stats.ttest_ind(mcts_gamma_data, monte_carlo_data, equal_var=False)
    p_sig = p < 0.05 and np.mean(mcts_gamma_data) > np.mean(monte_carlo_data)
    table_latex_sig_mc[model_name].append(p_sig)

    table_latex_sig_gamma[model_name] = []
    _, p = stats.ttest_ind(monte_carlo_data, max_mcts_gamma_data, equal_var=False)
    p_sig = p < 0.05 and np.mean(monte_carlo_data) > np.mean(max_mcts_gamma_data)
    table_latex_sig_gamma[model_name].append(p_sig)
    _, p = stats.ttest_ind(max_mcts_lambda_data, max_mcts_gamma_data, equal_var=False)
    p_sig = p < 0.05 and np.mean(max_mcts_lambda_data) > np.mean(max_mcts_gamma_data)
    table_latex_sig_gamma[model_name].append(p_sig)
    table_latex_sig_gamma[model_name].append(False)
    _, p = stats.ttest_ind(mcts_lambda_data, max_mcts_gamma_data, equal_var=False)
    p_sig = p < 0.05 and np.mean(mcts_lambda_data) > np.mean(max_mcts_gamma_data)
    table_latex_sig_gamma[model_name].append(p_sig)
    table_latex_sig_gamma[model_name].append(False)
    table_latex_sig_gamma[model_name].append(False)
    _, p = stats.ttest_ind(mcts_gamma_data, max_mcts_gamma_data, equal_var=False)
    p_sig = p < 0.05 and np.mean(mcts_gamma_data) > np.mean(max_mcts_gamma_data)
    table_latex_sig_gamma[model_name].append(p_sig)

print sorted(pd.unique(data.model_name.ravel()))
headers = [
    "Monte Carlo",
    "MaxMCTS($\\lambda$)",
    "$\\lambda$",
    "MCTS($\\lambda$)",
    "$\\lambda$",
    "MaxMCTS$_\\gamma$",
    "MCTS$_\\gamma$",
]
for row in range(7):
    print headers[row] + " &",
    for i, model_name in enumerate(sorted(pd.unique(data.model_name.ravel()))):
        modifier_str = ""
        if table_latex_sig_mc[model_name][row] and table_latex_sig_gamma[model_name][row]:
            modifier_str = "$^{\\ast\\dagger}$"
        elif table_latex_sig_mc[model_name][row]:
            modifier_str = "$^{\\ast}$"
        elif table_latex_sig_gamma[model_name][row]:
            modifier_str = "$^{\\dagger}$"
        print str(round(table_latex_data[model_name][row], 2)) + modifier_str,
        if i != num_models - 1:
            print "& ",
    print " \\\\"

