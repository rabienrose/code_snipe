import os
import sys
from sys import argv
file_name=argv[1]
entry_key=argv[2]
entry_value=argv[3]
print file_name
print entry_key
print entry_value
with open(file_name,"r") as f:
    lines = f.readlines() 

with open(file_name,"w") as f_w:
    for line in lines:
        if entry_key in line:
        	if "," in lines:
        		line = '"'+entry_key+'" : "'+entry_value+'",\n'
        	else:
        		line = '"'+entry_key+'" : "'+entry_value+'"\n'
        f_w.write(line)

		
		