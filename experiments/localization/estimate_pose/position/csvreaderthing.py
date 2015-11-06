import csv
from math import hypot
from glob import glob

#Extracts meaningful information from csv
def extract(filepath):
	newlist = []

	with open(filepath,"rb") as csvfile:
	
		f = csv.reader(csvfile,delimiter=",")

		f.next()
		x = f.next()
		top = int(x[0].split(".")[0])
		
		odomx = float(x[1])
		odomy = float(x[2])
		estx = float(x[3])
		esty = float(x[4])
		
		newlist.append([0,hypot(odomx - estx,odomy - esty)])		
				
		for row in f:
			if row[0] != "time":
				row[0] = int(row[0].split(".")[0]) - top
			
			odomx = float(row[1])
			odomy = float(row[2])
			estx = float(row[3])
			esty = float(row[4])
		
			#print(str(odomx) + "; " + str(odomy) + "; " + str(estx) + "; " + str(esty))
		
			newlist.append([row[0],hypot(odomx - estx,odomy - esty)])
		
		return newlist

def printlist(l):
	for x in l:
		print(x)

def collapselist(l):
	d = {}

	ab = []
	bc = []

	for row in l:
		ab.append(row[0])
		bc.append(row[1])

	for a, b in zip(ab, bc):
		d.setdefault(a, []).append(b)

	return d

#Prints the average of list m to file where a is the prefix
def printtofile(a,m):
	m = collapselist(m)
	
	with open("output/"+a+"meth.csv","w") as newfile:
		a = csv.writer(newfile)
		a.writerow(['time','diff'])
	
		for key in m:
			avg = sum(m[key])/len(m[key])
	
			a.writerow([key, avg])

if __name__ == '__main__':
	av = glob('*/*.csv')
	avgmeth = []
	distmeth = []
	groupmeth = []

	for f in av:
		if "output" in f:
			continue
	
		if "point" in f:
			distmeth = distmeth + extract(f)
		elif "average" in f:
			avgmeth = avgmeth + extract(f)
		elif "grouping" in f:
			groupmeth = groupmeth + extract(f)
		else:
			print "huh?"
		
	if avgmeth:
		printtofile('avgTEST',avgmeth)
	else:
		print "No data in avgmeth"
		
	if distmeth:
		printtofile('distTEST',distmeth)
	else:
		print "No data in distmeth"
		
	if groupmeth:
		printtofile('groupTEST',groupmeth)
	else:
		print "No data in groupmeth"
