from world_dom import CreateFromDocument
import urllib.request
import re
import sys
import os
import math


world_location = sys.argv[1]

root = CreateFromDocument(open(world_location).read())

world = root

print(len(world.model))

# create a text file
localfile = open("//Assets//temp" + world.name + "World.txt" ,'w')

models = world.model

localfile.write("world_name;")
localfile.write(world.name + "\n")


for i in range(0, len(models)):
	currentmodel = models[i]
	localfile.write("model_name;")
	localfile.write(currentmodel.name + ";")
	if(len(currentmodel.pose) > 0):
		localfile.write("model_pose;")
		localfile.write(currentmodel.pose[0] + ";")
	for j in range(0, len(currentmodel.link)):
		links = currentmodel.link
		
		# VIUSAL
		if(len(links[j].visual) > 0):
			VIS_model = links[j].visual[0]
			VIS_geometry = VIS_model.geometry[0]
			VIS_mesh = VIS_geometry.mesh
			if(len(VIS_mesh) > 0):
				localfile.write("model_link;")
				localfile.write(links[j].name + ";")
				if(len(links[j].pose)> 0 ):
					localfile.write("link_pose;")
					localfile.write(links[j].pose[0] + ";")
			
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
		COL_model = links[j].collision[0]
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