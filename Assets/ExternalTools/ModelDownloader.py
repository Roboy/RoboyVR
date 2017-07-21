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

# Search url for all possible links
pattern = re.compile('class="js-navigation-open" id=".*" title=".*"')
linkList = pattern.findall(string)
linkList = list(set(linkList))
directoryList = list()


for link in linkList:
	if "." in link:
		directoryList.append(link)

print("Found " ,len(directoryList), " directories")

# search for titles
titlePattern = re.compile('title=".*?"')

# find all title patterns
titleList = titlePattern.findall(str(directoryList).strip('[]'))


# get the content without "title=**"
contentPattern = re.compile('".*?"')
titleListFinal = contentPattern.findall(str(titleList).strip('[]'))

filelist = list()

# remove quotation marks
for i in range (0, len(titleListFinal)):
	filelist.append(re.sub('"', '', titleListFinal[i]));

print(str(filelist).strip('[]'))

xmllist = list()

for file in filelist:
	if (".world" in file) or (".sdf" in file):
		xmllist.append(file)

modellist = list()		

# remove entries which are neither .dae or .STL
for file in filelist:
	if (".dae" in file)	or (".STL" in file) or (".stl" in file):
		modellist.append(file)
		
			
			
		
print(str(modellist).strip('[]'))
print(str(xmllist).strip('[]'))

#############################################################
export_list = []
tempArr = list()

export_Wlist = []
tempArrXML = list()

for model in modellist:
	tempArr.append(re.sub(' ', '%20', model))
	
for xml in xmllist:
	tempArrXML.append(re.sub(' ', '%20', xml))

print(str(tempArr).strip('[]'))

if sys.argv[5] == "":
	export_list = tempArr
	export_XMLlist = tempArrXML
else:
	#compare arguments with list of directory in github
	for filename in tempArr:	
		if filename in stl_arguments:
			export_list.append(filename)


print("Meshes to be updated: ", export_list)


#Download all (.world)s located at GitHub
for filename in export_XMLlist:
	print("Downloading: " + filename + " from " + dir_ip_addr + filename)
	remotefile = urllib.request.urlopen(dir_ip_addr + filename)
	if not os.path.exists(pathToProjectModels):
		os.makedirs(pathToProjectModels)
	localfile = open(pathToProjectModels + "/" + filename,'wb')
	localfile.write(remotefile.read())
	localfile.close()
	remotefile.close()

#Download all (.dae/.STL)s located at GitHub
for filename in export_list:
	print("Downloading: " + filename + " from " + dir_ip_addr + filename)
	remotefile = urllib.request.urlopen(dir_ip_addr + filename)
	if not os.path.exists(pathToProjectModels):
		os.makedirs(pathToProjectModels)
	localfile = open(pathToProjectModels + "/" + filename,'wb')
	localfile.write(remotefile.read())
	localfile.close()
	remotefile.close()

#Start progress of conversion

for filename in export_list:
#Import STL or dae
	print("Importing " + filename)
	
	if ".dae" in filename: 
 		bpy.ops.wm.collada_import(filepath = pathToProjectModels + "\\" + filename)
		
	if (".STL" in filename) or (".stl" in filename):
		print("Found STL: "+ filename)
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
