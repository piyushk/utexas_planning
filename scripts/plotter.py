#!/usr/bin/env python

import argparse
import graph
import filesystem as fs
import json
import matplotlib.pyplot as plt; plt.rcdefaults()
from matplotlib.font_manager import FontProperties

parser = argparse.ArgumentParser()
parser.add_argument("data", help="The directory containing csv files or a single csv file.", type=str)
parser.add_argument("output", help="Column name in csv to plot.", type=str)
parser.add_argument("--plot-type", help="Plot type. One of ['line', 'bar', '3d'].", type=str)
parser.add_argument("--legend-loc", help="Legend location. 'none' removes legend, and 'best' tries to auto-place it. 2 word combination of ['upper','lower','center','left','right'].", type=str, default='best')
parser.add_argument("--expand-legend", help="Expand legend across entire plot", action='store_true')
parser.add_argument("--legend-cols", help="Expand legend across entire plot", type=int, default=2)
parser.add_argument("--filter", help="Comma separated primary grouping filter. Defaults to 'planner_name' if available",
                    type=str)
parser.add_argument("--secondary-filter", help="Comma separated secondary grouping filter.", type=str)
parser.add_argument("--name-mappings", help="JSON string with mapping from column name to printed name.", type=str)
parser.add_argument("--attempt-auto-mapping", help="Attempt to perform automatic cleanup of CSV column name.",
                    action='store_true')
parser.add_argument("--write-image", help="Write an image, and ask questions to cleanup image", action='store_true')
parser.add_argument("--image-width", help="Image width (smaller means larger text)", type=float, default=5)
parser.add_argument("--image-height", help="Image height (smaller means larger text)", type=float, default=4)
parser.add_argument("--no-error-bars", help="Don't draw error bars", action='store_true')
parser.add_argument("--one-error-bar", help="Only draw error bars for specified line idx", type=int, default=-1)

args = parser.parse_args()

if not args.plot_type:
    args.plot_type = 'bar'

if not args.filter:
    args.filter = 'planner_name'

name_mappings = None
if args.name_mappings is not None:
    name_mappings = json.loads(args.name_mappings)

fig, ax, rects, means, sigs = \
        graph.draw_from_data_frame(fs.expand_path_to_filelist(args.data), args.output, args.plot_type, args.filter,
                                   args.secondary_filter, args.attempt_auto_mapping, name_mappings, args)

font0 = FontProperties()

if args.plot_type == "bar":
    for i in range(len(rects)):
        for j in range(len(rects[i])):
            height = rects[i][j].get_height()
            if means[i][j] < 0:
                height = -height
            font = font0.copy()
            if sigs[i][j]:
                font.set_weight('bold')
            ax.text(rects[i][j].get_x()+rects[i][j].get_width()/2., 1.03*height, "%.1f"%means[i][j], ha='center',
                    va='center', fontproperties=font)

highest = [0] * len(means)

for i in range(len(means)):
    for j in range(len(means[i])):
        if means[i][j] > means[i][highest[i]]:
            highest[i] = j

for j in range(len(means[0])):
    for i in range(len(means)):
        # if sigs[i][j] and highest[i] == j:
        #     print "\\textit{",
        print "%.2f"%means[i][j],
        # if sigs[i][j] and highest[i] == j:
        #     print "}",
        if i != len(means) - 1: # or i != len(means) - 1:
            print " & ",
        else:
            print "\\\\"

if args.write_image:
    fig = plt.gcf()
    fig.set_size_inches(args.image_width, args.image_height)
    pad_inches = 0.1
    if args.plot_type == "3d":
        azim = raw_input("Enter Azimuth [Hit enter to default to -70]: ")
        if azim is None or azim == "":
            azim = -70
        elev = raw_input("Enter Elevation [Hit enter to default to 25]: ")
        if elev is None or elev == "":
            elev = 25
        ax.view_init(elev=float(elev), azim=float(azim))

    plt.savefig('out.png',bbox_inches='tight',pad_inches=pad_inches,dpi=100)

#plt.setp(ltext, fontsize='small')

plt.show()
