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
localfile = open("Assets//ExternalTools//tempModelSDFs.txt" ,'w')

links = robot.link

#for link in links
#i = 1
for i in range(0, len(links)):
#while i <= len(robot.link)
	print(i)
	localfile.write("Model Link = ")
	localfile.write(links[i].name + "\n")
	
	localfile.write("Link Pose = ")
	localfile.write(links[i].pose[0] + "\n")
	
	
	# VIUSAL
	VIS_model = links[i].visual[0]
	#print(VIS_model.name)
	VIS_geometry = VIS_model.geometry[0]
	#if mesh in VIS_geometry 
	#print("VIS_geometry has attributes")
	VIS_mesh = VIS_geometry.mesh
	#print(len(VIS_mesh))
	#if hasattr(VIS_mesh, 'scale'):
	if(len(VIS_mesh) > 0):
		localfile.write("visual mesh uri = " + VIS_geometry.mesh[0].uri[0] + "\n")
		localfile.write("visual mesh scale = " + VIS_geometry.mesh[0].scale[0] + "\n")
		print("mesh link and scale saved")
	VIS_box = VIS_geometry.box
	if(len(VIS_box) > 0):
		localfile.write("visual box size = " + VIS_geometry.box[0].size[0] + "\n")
		print("box size saved")
	VIS_sphere = VIS_geometry.sphere
	if(len(VIS_sphere) > 0):
		localfile.write("visual sphere radius = " + str(VIS_geometry.sphere[0].radius[0]) + "\n")
		print("box radius saved")
		
		
	# COLLISION
	COL_model = links[i].visual[0]
	COL_geometry = COL_model.geometry[0]
	COL_mesh = COL_geometry.mesh
	#print(len(COL_mesh))
	#if hasattr(COL_mesh, 'scale'):
	if(len(COL_mesh) > 0):
		localfile.write("collision mesh uri = " + COL_geometry.mesh[0].uri[0] + "\n")
		localfile.write("collision mesh scale = " + COL_geometry.mesh[0].scale[0] + "\n")
		print("mesh link and scale saved")
	COL_box = COL_geometry.box
	if(len(COL_box) > 0):
		localfile.write("collision box size = " + COL_geometry.box[0].size[0] + "\n")
		print("box size saved")
	COL_sphere = COL_geometry.sphere
	if(len(COL_sphere) > 0):
		localfile.write("collision sphere radius = " + str(COL_geometry.sphere[0].radius[0]) + "\n")
		print("box radius saved")
		
	


localfile.close()