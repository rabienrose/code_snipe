import csv
import sys
csvName = sys.argv[1]
caseName = sys.argv[2]
with open(csvName, newline='') as csvfile:
    spamreader = csv.reader(csvfile)
    for row in spamreader:
        if row[0]==caseName:
            print('{0:s} {1:s} {2:s}'.format(row[1], row[2], row[3]))
            break
