from sdf_dom import CreateFromDocument
import urllib.request
import re
import sys
import os


sdf_location = sys.argv[1]

root = CreateFromDocument(open(sdf_location).read())

robot = root.model[0]

# create a text file
localfile = open("Assets//ExternalTools//tempModelSDFs.txt" ,'w')

# remove quotation marks
localfile.write("Model Name = ")
# remove quotation marks and insert github link
localfile.write(robot.name + "\n")


localfile.close()