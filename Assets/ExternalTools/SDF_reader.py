from sdf_dom import CreateFromDocument
import urllib.request
import re
import sys
import os
import math


sdf_location = sys.argv[1]

root = CreateFromDocument(open(sdf_location).read())

robot = root.model[0]

print(len(robot.link))

# create a text file
localfile = open("Assets//temp" + robot.name + "SDFs.txt" ,'w')

links = robot.link

localfile.write("model_name;")
localfile.write(robot.name + "\n")
#for link in links
#i = 1
for i in range(0, len(links)):
#while i <= len(robot.link)
	print(i)
	# VIUSAL
	VIS_model = links[i].visual[0]
	VIS_geometry = VIS_model.geometry[0]
	VIS_mesh = VIS_geometry.mesh
	if(len(VIS_mesh) > 0):
		localfile.write("model_link;")
		localfile.write(links[i].name + ";")
		localfile.write("link_pose;")
		localfile.write(links[i].pose[0] + ";")
	
		localfile.write("VIS_mesh_uri;" + VIS_geometry.mesh[0].uri[0] + ";")
		if(len(VIS_geometry.mesh[0].scale) > 0):
			localfile.write("VIS_mesh_scale;" + VIS_geometry.mesh[0].scale[0] + ";")
		print("mesh link and scale saved")
	# VIS_box = VIS_geometry.box
	# if(len(VIS_box) > 0):
		# localfile.write("VIS_box_size;" + VIS_geometry.box[0].size[0] + "\n")
		# print("box size saved")
	# VIS_sphere = VIS_geometry.sphere
	# if(len(VIS_sphere) > 0):
		# localfile.write("VIS_sphere_radius;" + str(VIS_geometry.sphere[0].radius[0]) + "\n")
		# print("box radius saved")


	# COLLISION
	COL_model = links[i].collision[0]
	COL_geometry = COL_model.geometry[0]
	COL_mesh = COL_geometry.mesh
	#print(len(COL_mesh))
	#if hasattr(COL_mesh, 'scale'):
	if(len(COL_mesh) > 0):
		localfile.write("COL_mesh_uri;" + COL_geometry.mesh[0].uri[0] + ";")
		if(len(COL_geometry.mesh[0].scale) > 0):
			localfile.write("COL_mesh_scale;" + COL_geometry.mesh[0].scale[0] + ";")
		print("mesh link and scale saved")
	# COL_box = COL_geometry.box
	# if(len(COL_box) > 0):
		# localfile.write("COL_box_size;" + COL_geometry.box[0].size[0] + "\n")
		# print("box size saved")
	# COL_sphere = COL_geometry.sphere
	# if(len(COL_sphere) > 0):
		# localfile.write("COL_sphere_radius;" + str(COL_geometry.sphere[0].radius[0]) + "\n")
		# print("box radius saved")
	if(len(VIS_mesh) > 0):
		localfile.write("\n")	
	


localfile.close()