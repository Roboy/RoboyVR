import urllib.request
import re
from math import pi
import sys
import os

#Define location to scan for RobotModels
dir_ip_addr = sys.argv[1]
urlpath = urllib.request.urlopen(sys.argv[1])
string = urlpath.read().decode('utf-8')

#Search url for all possible links
pattern = re.compile('href=".*" class="js-navigation-open" id=".*" title=".*"')
linkList = pattern.findall(string)
linkList = list(set(linkList))
directoryList = list()

for link in linkList:
	if "." not in link:
		directoryList.append(link)

print("Found " ,len(directoryList), " directories")

titlePattern = re.compile('title=".*?"')
linkPattern = re.compile('href=".*?"')


titleList = titlePattern.findall(str(directoryList).strip('[]'))
dirLinkList = linkPattern.findall(str(directoryList).strip('[]'))

for title in titleList:
	print(title)
	
for dirLink in dirLinkList:
	print(dirLink)
