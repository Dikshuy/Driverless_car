import ast
import os
import sys
import random


if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib
from sumolib.visualization import helpers
import matplotlib
import matplotlib.pyplot as plt
from optparse import OptionParser
from matplotlib import cm



directory = '/home/dimple/sumo_ws/src/patrol_algo/heatmap_files/'
save_path = '/home/dimple/sumo_ws/src/patrol_algo/heatmap_results/'
path = '/home/dimple/sumo_ws/src/patrol_algo/graph_sumo/Ladder.net.xml'

for filename in os.listdir(directory):
	if filename.endswith(".txt"): 
		file_path = os.path.join(directory, filename)
		robot_id = str(file_path.split('/')[-1].split('.')[0].split('_')[0])
		print (robot_id)
		edge_traversal_list = {}
		with open(file_path, 'r') as data:
			for line in data:
				edge_traversal_list = ast.literal_eval(line)

		#print (edge_traversal_list)
		data.close()
		args=None

		optParser = OptionParser()
		optParser.add_option("-n", "--net", dest="net", metavar="FILE", help="Defines the network to read")
		optParser.add_option("--edge-width", dest="defaultWidth", type="float", default=2.5, help="Defines the edge width")
		optParser.add_option("--edge-color", dest="defaultColor", default="r", help="Defines the edge color")

		optParser.add_option(
			"-v",
			"--verbose",
			dest="verbose",
			action="store_true",
			default=False,
			help="If set, the script says what it's doing",
		)
		# standard plot options
		helpers.addInteractionOptions(optParser)
		helpers.addPlotOptions(optParser)
		options, remaining_args = optParser.parse_args(args=args)
		options.colormap = "jet"
		#Net file

		net = sumolib.net.readNet(path)
		#count = 0

		key_max = max(edge_traversal_list.keys(), key=(lambda k: edge_traversal_list[k]))
		key_min = min(edge_traversal_list.keys(), key=(lambda k: edge_traversal_list[k]))
		minCount = edge_traversal_list[key_min]
		maxCount = edge_traversal_list[key_max]

		helpers.linNormalise(edge_traversal_list, minCount, maxCount)

		for e in edge_traversal_list:
			edge_traversal_list[e] = helpers.getColor(options, edge_traversal_list[e], 1.)

		fig, ax = helpers.openFigure(options)
		str_val = 'Edge Traversal count for robot "{}"'
		fig.suptitle(str_val.format(robot_id), fontsize=12)

		fig.patch.set_facecolor('white')
		fig.patch.set_visible(True)
		ax.axis('off')
		ax.set_aspect("equal", None, 'C')\

		helpers.plotNet(net, edge_traversal_list, {}, options)

		sm = matplotlib.cm.ScalarMappable(cmap=matplotlib.cm.jet)

		sm._A = []
		plt.tight_layout()

		plt.colorbar(sm)
		options.nolegend = True

		str_val = save_path+'{}_edge_traverse_heatmap.png'
		plt.savefig(str_val.format(robot_id))
		helpers.closeFigure(fig, ax, options)

