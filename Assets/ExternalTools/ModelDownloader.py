#!/usr/bin/python

import urllib.request
import bpy
import re
from math import pi
import sys
import os

def areas_tuple():
    res = {}                                                               
    count = 0
    for area in bpy.context.screen.areas:                                  
        res[area.type] = count                                             
        count += 1
    return res
	
def get_override(area_type, region_type):
    for area in bpy.context.screen.areas: 
        if area.type == area_type:             
            for region in area.regions:                 
                if region.type == region_type:                    
                    override = {'area': area, 'region': region} 
                    return override
    #error message if the area or region wasn't found
    raise RuntimeError("Wasn't able to find", region_type," in area ", area_type,
                    "\n Make sure it's open while executing script.")

#pathToRoboyModels = r"D:\RoboyVR\Assets\SimulationModels\Test\OriginModels"
pathToProjectModels = sys.argv[4]
					
	
#we expect the array to be of format <name>
arguments = sys.argv[5].split(',')
stl_arguments = []

for argument in arguments:
	stl_arguments.append(argument + ".STL")
	



#Define location to download STls from
dir_ip_addr = sys.argv[3]
urlpath = urllib.request.urlopen(sys.argv[3])
string = urlpath.read().decode('utf-8')

#Search url for all STL names
pattern = re.compile('[a-z]+_?[a-z]*[0-9]*.STL')
filelist = pattern.findall(string)
filelist = list(set(filelist))

export_list = []

if sys.argv[5] == "":
	export_list = filelist
else:
	#compare arguments with list of directory in github
	for filename in filelist:	
		if filename in stl_arguments:
			export_list.append(filename)

print("Meshes to be updated: ", export_list)

#Download all STLs located at GitHub
for filename in export_list:
	print("Downloading: ", filename)
	remotefile = urllib.request.urlopen(dir_ip_addr + filename)
	localfile = open(pathToProjectModels + "\\" + filename,'wb')
	localfile.write(remotefile.read())
	localfile.close()
	remotefile.close()

#Start progress of conversion

for filename in export_list:
#Import STL
	print("Importing " + filename)
	bpy.ops.import_mesh.stl(filepath = pathToProjectModels + "\\" + filename)
	ob = bpy.context.object
	#Rotate Mesh
	ob.rotation_euler = (pi/2, 0, 0)
	bpy.ops.object.mode_set(mode='EDIT', toggle = False)
	areas = areas_tuple()
	bpy.context.screen.areas[areas['VIEW_3D']].spaces[0].pivot_point = 'CURSOR'
	bpy.context.screen.areas[areas['VIEW_3D']].spaces[0].cursor_location = (0.0,0.0,0.0)

	override = get_override( 'VIEW_3D', 'WINDOW' )
	bpy.ops.mesh.select_all(action='SELECT')
	bpy.ops.transform.rotate(override, value=-pi/2, axis=(1, 0, 0))

	bpy.ops.object.mode_set(mode = 'OBJECT')

	#Export FBX
	name_fbx = "./" + filename.split('.')[0]
	print("Exporting " + name_fbx)
	bpy.ops.export_scene.fbx(filepath = pathToProjectModels + "\\" + name_fbx + '.fbx')
	
	#Delete current object
	bpy.ops.object.delete()
	filepath = pathToProjectModels + "\\" + filename
	os.remove(filepath)

sys.exit("EXIT")
