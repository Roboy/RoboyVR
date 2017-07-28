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
	localfile.write("Model Link = ")
	localfile.write(links[i].name + "\n")
	
	localfile.write("Link Pose = ")
	localfile.write(links[i].pose[0] + "\n")
	
	VIS_model = links[i].visual[0]
	VIS_geometry = VIS_model.geometry[0]
	#if mesh in VIS_geometry 
	try:
		VIS_geometry.mesh[0]
	except NameError:
		print("NameError")
	else:
		print("Yes")
		#if(hasattr(VIS_geometry, 'mesh'))
		#VIS_mesh = VIS_geometry.mesh
		#localfile.write("Visual uri = " + VIS_geometry.mesh.uri + "\n")
		
	print(i)


localfile.close()