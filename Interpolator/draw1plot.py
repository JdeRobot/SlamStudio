import numpy as np
import matplotlib.pyplot as plt
import sys
import numpy
import argparse




#leer archivo
def read_trajectory(filename, matrix=True):
    """
    Read a trajectory from a text file. 
    
    Input:
    filename -- file to be read
    matrix -- convert poses to 4x4 matrices
    
    Output:
    New data file , based on Input file
    """
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n") 
    list = [[float(v.strip()) for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
    list_ok = []
    for i,l in enumerate(list):
    	print l
    	#l[2]=l[2]*0.5  # We make a modification on the ground-truth data to get a new dataset based on the ground-truth . In this example field2 = field2*05 . Format of file = timestamp tx ty tz qx qy qz qw
    	list_ok.append(l)
    traj = list_ok
    #    if l[4:8]==[0,0,0,0]:
    #        continue
    #    isnan = False
    #    for v in l:
    #        if numpy.isnan(v): 
    #            isnan = True
    #            break
    #    if isnan:
    #        sys.stderr.write("Warning: line %d of file '%s' has NaNs, skipping line\n"%(i,filename))
    #        continue
    #    list_ok.append(l)
    #if matrix :
    #  traj = dict([(l[0],transform44(l[0:])) for l in list_ok])
    #else:
    #  traj = dict([(l[0],l[1:8]) for l in list_ok])
      
    return traj



#pintar datos

parser = argparse.ArgumentParser(description='''
    This script computes the relative pose error from the ground truth trajectory and the estimated trajectory. 
    ''')
parser.add_argument('orig_file', help='ground-truth trajectory file (format: "timestamp tx ty tz qx qy qz qw")')

args = parser.parse_args()
print ("Reading "+args.orig_file)

traj_gt = read_trajectory(args.orig_file,False) # reads trajectory from groundTruth and performs small changes

miLen = len(traj_gt)
t_ok = []
x_ok = []
for j in range (miLen):
	#j=j+1
	print "j",j
	print traj_gt[j][0]
	t_ok.append (traj_gt[j][0])
	x_ok.append (traj_gt[j][1])
print "t_ok",t_ok
print "tsize",len(traj_gt)
t= np.asarray(t_ok)
x= np.asarray(x_ok)




#plt.plot(t, x, 'r--', t, x**2, 'bs', t, x**3, 'g^')
#plt.plot(t, x, 'r--', t2, x2, 'b--')
#plt.plot(x, t, 'r' )
plt.plot(t, x, 'r.' )

plt.show()


# evenly sampled time at 200ms intervals
#t = np.arange(0., 5., 0.2)

# red dashes, blue squares and green triangles
#plt.plot(t, t, 'r--', t, t**2, 'bs', t, t**3, 'g^')
#plt.show()
