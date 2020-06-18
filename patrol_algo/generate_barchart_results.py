import ast
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt; plt.rcdefaults()
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
import os

directory = '/home/dimple/sumo_ws/src/patrol_algo/heatmap_files/'
save_path = '/home/dimple/sumo_ws/src/patrol_algo/heatmap_results/'

for filename in os.listdir(directory):
	if filename.endswith(".txt"): 
		file_path = os.path.join(directory, filename)
		robot_id = str(file_path.split('/')[-1].split('.')[0].split('_')[0])
		print (robot_id)
		edge_traversal_list = {}
		with open(file_path, 'r') as data:
			for line in data:
		  		edge_traversal_list = ast.literal_eval(line)
		data.close()
		#print (edge_traversal_list)

		edge_id = []
		edge_count = []
		for key, value in edge_traversal_list.items():
			#print (key, value)
			edge_id.append(key)
			edge_count.append(value)

		#print (edge_id, len(edge_id), len(edge_count))

		y_pos = np.arange(len(edge_count))

		figure = plt.gcf() # get current figure
		figure.set_size_inches(10, 10)
		plt.barh(y_pos, edge_count, align='center', alpha=0.5)
		plt.yticks(y_pos, edge_id)
		plt.ylabel('Edge IDs')
		plt.xlabel('Count(Number of times the edge is visited)')

		str_val = 'Edge traversal for robot {}'
		plt.title(str_val.format(robot_id))
		str_val = save_path+'{}_edge_traverse_barchart.png'
		plt.savefig(str_val.format(robot_id), bbox_inches='tight',dpi=300)
		plt.show()


