import urllib.request
import re
from math import pi
import sys
import os

# Define location to scan for RobotModels
dir_ip_addr = sys.argv[1]
urlpath = urllib.request.urlopen(sys.argv[1])
string = urlpath.read().decode('utf-8')

# Search url for all possible links
pattern = re.compile('href=".*" class="js-navigation-open" id=".*" title=".*"')
linkList = pattern.findall(string)
linkList = list(set(linkList))
directoryList = list()

# Get all links to directories, it should work because directories do not have an "." extension (f.e ".md")
# will crash if there is a file without an extension 
for link in linkList:
	if "." not in link:
		directoryList.append(link)

print("Found " ,len(directoryList), " directories")

# search for titles and the corresponding links to the model directory
titlePattern = re.compile('title=".*?"')
linkPattern = re.compile('href=".*?"')
# find all title and link patterns
titleList = titlePattern.findall(str(directoryList).strip('[]'))
dirLinkList = linkPattern.findall(str(directoryList).strip('[]'))

# get the content without "title=**" or "href=**"
contentPattern = re.compile('".*?"')
titleListFinal = contentPattern.findall(str(titleList).strip('[]'))
dirLinkListFinal = contentPattern.findall(str(dirLinkList).strip('[]'))

# create a text file
localfile = open("Assets//tempModelURLs.txt" ,'w')

# write to file
for i in range(0, len(titleListFinal)):
	print(titleListFinal[i])
	print(dirLinkListFinal[i])
	# remove quotation marks
	localfile.write(re.sub('"', '', titleListFinal[i]) + ";")
	# remove quotation marks and insert github link
	localfile.write("https://github.com" + re.sub('"', '', dirLinkListFinal[i]) + "\n")
	

localfile.close()