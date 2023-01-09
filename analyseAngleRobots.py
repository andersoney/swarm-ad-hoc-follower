import sys
import numpy as np
import matplotlib.pyplot as plt
import math
import os
import subprocess
from scipy.stats import t
from array import array


def calcConfInt(mean,var,size):
  alpha = 0.01
  df = size - 1
  unbiased_sd = math.sqrt((size/df)*var)
  t_variate = t.ppf(1-alpha/2,df)
  uppervalue = mean + t_variate * unbiased_sd/math.sqrt(size)
  return uppervalue


prefixDirectory="./new adhoc experiments/"
algorithmsLabels = ["SQF","TRVF"];
algorithmsSymbol = ["."]*len(algorithmsLabels);
varValues = range(15, 91, 15)
nSamples = [40]*len(algorithmsLabels);
datalines = 19+1
holoSubStr = ['nonholo','holo']
numRobotsList = [100,200]
probList = [10,50,90]
data = np.zeros((len(varValues),len(algorithmsLabels),max(nSamples),len(holoSubStr),len(numRobotsList),len(probList),datalines));
dataMean = np.zeros((len(varValues),len(algorithmsLabels),len(holoSubStr),len(numRobotsList),len(probList),datalines));
dataVari = np.zeros((len(varValues),len(algorithmsLabels),len(holoSubStr),len(numRobotsList),len(probList),datalines));
dataUpCi = np.zeros((len(varValues),len(algorithmsLabels),len(holoSubStr),len(numRobotsList),len(probList),datalines));
printValuesForTTest = False # Set true if one wishes to print values for t-test.

'''
Results from t-test:

SQF nonholo n=100 p=90 0 3
[(False, 0.008070395404523367)]
SQF nonholo n=100 p=90 0 4
[(False, 4.801456254810432e-08)]
SQF nonholo n=100 p=90 0 5
[(False, 1.9771222037334724e-07)]
SQF nonholo n=100 p=90 1 4
[(False, 3.0932188383570036e-05)]
SQF nonholo n=100 p=90 1 5
[(False, 4.12425514899617e-05)]
SQF nonholo n=100 p=90 2 4
[(False, 2.1532361570431036e-05)]
SQF nonholo n=100 p=90 2 5
[(False, 2.5904083156058633e-05)]
SQF nonholo n=100 p=90 3 4
[(False, 0.001551303565831219)]
SQF nonholo n=100 p=90 3 5
[(False, 0.0011341364437644774)]

SQF holo n=100 p=10 0 1
[(False, 0.00435705973994116)]
SQF holo n=100 p=10 0 2
[(False, 0.0001175025339734681)]
SQF holo n=100 p=10 0 3
[(False, 2.022267150691981e-05)]
SQF holo n=100 p=10 0 4
[(False, 0.0009727856831180937)]
SQF holo n=100 p=10 0 5
[(False, 0.00012972106532549432)]

SQF holo n=100 p=90 0 2
[(False, 0.0031546080433060286)]
SQF holo n=100 p=90 0 4
[(False, 2.4466632690955947e-06)]
SQF holo n=100 p=90 0 5
[(False, 0.0005166517463457421)]
SQF holo n=100 p=90 1 4
[(False, 8.011111543249072e-06)]
SQF holo n=100 p=90 1 5
[(False, 0.0021268612377760565)]
SQF holo n=100 p=90 3 4
[(False, 0.0023958037394120613)]

SQF nonholo n=200 p=50 0 3
[(False, 6.133379727057964e-05)]
SQF nonholo n=200 p=50 0 4
[(False, 1.9522170669583616e-05)]
SQF nonholo n=200 p=50 0 5
[(False, 3.384590738786386e-07)]
SQF nonholo n=200 p=50 1 3
[(False, 3.963449118793427e-05)]
SQF nonholo n=200 p=50 1 4
[(False, 1.2757383576644798e-05)]
SQF nonholo n=200 p=50 1 5
[(False, 2.1333497723929895e-07)]
SQF nonholo n=200 p=50 2 5
[(False, 0.0029171521490218844)]

SQF nonholo n=200 p=90 0 1
[(False, 0.004917137353020129)]
SQF nonholo n=200 p=90 0 3
[(False, 0.0008741696026113299)]
SQF nonholo n=200 p=90 0 4
[(False, 2.6508310568829074e-07)]
SQF nonholo n=200 p=90 0 5
[(False, 0.001722996808674493)]
SQF nonholo n=200 p=90 2 4
[(False, 0.004901523497647942)]

SQF holo n=200 p=10 0 5
[(False, 0.008673359015515913)]

SQF holo n=200 p=50 0 3
[(False, 0.0061821526477392474)]
SQF holo n=200 p=50 0 4
[(False, 0.0035708471601010316)]
SQF holo n=200 p=50 0 5
[(False, 7.674881627406371e-05)]
SQF holo n=200 p=50 1 4
[(False, 0.006258470257354487)]
SQF holo n=200 p=50 1 5
[(False, 9.361446000411e-05)]
SQF holo n=200 p=50 2 5
[(False, 0.0003941705552235053)]

TRVF nonholo n=100 p=10 2 5
[(False, 0.0004477690150133107)]
TRVF nonholo n=100 p=10 3 5
[(False, 0.005068796206845638)]

TRVF nonholo n=100 p=90 0 4
[(False, 3.506476893622157e-05)]
TRVF nonholo n=100 p=90 0 5
[(False, 1.0293952058315625e-05)]
TRVF nonholo n=100 p=90 1 4
[(False, 0.00020851824473111513)]
TRVF nonholo n=100 p=90 1 5
[(False, 9.243346805853925e-05)]
TRVF nonholo n=100 p=90 2 4
[(False, 0.0018883856463196569)]
TRVF nonholo n=100 p=90 2 5
[(False, 0.0012428855099109182)]

TRVF holo n=100 p=90 0 4
[(False, 0.000820241657899512)]
TRVF holo n=100 p=90 0 5
[(False, 0.00152853846985046)]
TRVF holo n=100 p=90 1 4
[(False, 0.000722884234827692)]
TRVF holo n=100 p=90 1 5
[(False, 0.0013450589841468297)]
TRVF holo n=100 p=90 2 4
[(False, 0.0002820104125178524)]
TRVF holo n=100 p=90 2 5
[(False, 0.000506283003956387)]

TRVF nonholo n=200 p=50 0 4
[(False, 0.00022204492354194194)]
TRVF nonholo n=200 p=50 0 5
[(False, 1.3881258805525931e-05)]
TRVF nonholo n=200 p=50 1 5
[(False, 0.004640402764515716)]
TRVF nonholo n=200 p=50 2 5
[(False, 0.0006318441321175872)]

TRVF holo n=200 p=10 0 3
[(False, 0.003924915116616745)]
TRVF holo n=200 p=50 0 3
[(False, 0.0018379938984063315)]
TRVF holo n=200 p=50 0 4
[(False, 0.0006876336554650919)]
TRVF holo n=200 p=50 0 5
[(False, 7.4038216029137516e-06)]
TRVF holo n=200 p=50 2 5
[(False, 0.006524547525908142)]
'''

for h in range(len(holoSubStr)):
  for nRob in range(len(numRobotsList)):
    for p in range(len(probList)):
      for n in range(len(varValues)):
        for a in range(len(algorithmsLabels)):
          algorithmLocation =  prefixDirectory+holoSubStr[h]+"/NoCoordAlt/"+algorithmsLabels[a]+"/robots_"+str(numRobotsList[nRob])+"/m_"+str(int(numRobotsList[nRob]*probList[p]/100))+"/neighbourhoodAngle_"+str(varValues[n])
          for s in range(nSamples[a]):
            dataFile = open(algorithmLocation+"/log_"+str(s));
            dataFileStr = dataFile.readlines();
            for option in range(datalines):
              if option == 19:
                FirstRobotReachingTimeline = 8 
                LastRobotReachingTimeline = 9
                data[n, a, s, h, nRob, p, option] = (numRobotsList[nRob]-1)/((float(dataFileStr[LastRobotReachingTimeline]) - float(dataFileStr[FirstRobotReachingTimeline]))/1e6);
              elif option in [11,12,13,14,16,17]:
                data[n, a, s, h, nRob, p, option] = float(dataFileStr[option]);
              elif option in [8,9,10]:
                data[n, a, s, h, nRob, p, option] = float(dataFileStr[option])/1e6;
              else:
                data[n, a, s, h, nRob, p, option] = int(dataFileStr[option]);
          for option in range(datalines):
            dataMean[n, a, h, nRob, p, option] = np.mean(data[n,a,:nSamples[a],h,nRob,p,option]);
            dataVari[n, a, h, nRob, p, option] = np.var(data[n,a,:nSamples[a],h,nRob,p,option]);
            if all(data[n,a,0,h,nRob,p,option] == rest for rest in data[n,a,:,h,nRob,p,option]):
              dataUpCi[n, a, h, nRob, p, option] = dataMean[n, a, h, nRob, p, option]
            else:
              dataUpCi[n, a, h, nRob, p, option] = calcConfInt(dataMean[n, a, h, nRob, p, option],dataVari[n, a, h, nRob, p, option],nSamples[a]);
  
list_line_ylabel = [ 
  #           Label                                  # index
  #--------------------------------------------------#-------
  "Total number of iterations",                      # 0
  "Total iterations of the last robot",              # 1
  "Number of messages",                              # 2
  "Summation of the iter. for reaching",             # 3
  "Summation of the iter. for exiting",              # 4
  "Last robot's iterations for reaching",            # 5
  "Last robot's iterations for exiting",             # 6
  "Stalls",                                          # 7
  "First robot's reaching time (s)",                 # 8
  "Last robot's reaching time (s)",                  # 9
  "Total time of the simulation (s)",                # 10
  "Minimum distance (m)",                            # 11
  "Maximum velocity (m/s)",                          # 12
  "Mean distance (m)",                               # 13
  "St. dev. of distance (m)",                        # 14
  "Number of samples for distance",                  # 15
  "Mean velocity (m/s)",                             # 16
  "St. dev. of velocity (m/s)",                      # 17
  "Number of samples for velocity",                  # 18
  "Throughput (1/s)"                                 # 19
]

plt.rcParams.update({'font.size': 20})

def plotLogs(a,h,nRob,p,option,nSamples,data,varValues):
  for s in range(nSamples[a]):
    plt.plot(varValues,data[:,a,s,h,nRob,p,option],color='orange',marker='x',linestyle="None");
    for i in range(len(varValues)):
      plt.annotate(s, (varValues[i], data[i,a,s,h,nRob,p,option]))
  plt.show()
  plt.clf()

def mainLoop(option):
  if printValuesForTTest:
    print('==== '+list_line_ylabel[option]+' ====')
  for a in range(len(algorithmsLabels)):
    for nRob in range(len(numRobotsList)):
      for h in range(len(holoSubStr)):
        for p in range(len(probList)):
          varValuesRad = [v*math.pi/180 for v in varValues]
          plt.errorbar(varValuesRad,dataMean[:,a,h,nRob,p,option], yerr=[m - n for m,n in zip(dataUpCi[:,a,h,nRob,p,option],dataMean[:,a,h,nRob,p,option])], label=algorithmsLabels[a]+" "+str(probList[p])+"%" ,marker=algorithmsSymbol[a],capsize=5);
          # ~ plotLogs(a,h,nRob,p,option,nSamples,data,varValues)
          if printValuesForTTest:
            print('#',end='')
            print(algorithmsLabels[a],holoSubStr[h],numRobotsList[nRob],probList[p],sep='-------')
            print('means'+algorithmsLabels[a]+' = ',end='')
            print(*dataMean[:,a,h,nRob,p,option], sep=', ')
            print('vari'+algorithmsLabels[a]+' = ',end='')
            print(*dataVari[:,a,h,nRob,p,option], sep=', ')
            textResult = algorithmsLabels[a]+" "+holoSubStr[h]+" n="+str(numRobotsList[nRob])+" p="+str(probList[p])
            print("printResult(means"+algorithmsLabels[a]+",vari"+algorithmsLabels[a]+",n1,n2,\""+textResult+"\")")
          plt.legend()
          plt.xlabel("Angle (rad)");
          plt.ylabel(list_line_ylabel[option])
        imgLocation = "AnglesRobots"+algorithmsLabels[a]+str(numRobotsList[nRob])+holoSubStr[h]+str(option)
        print(imgLocation + ".pdf generated.")
        plt.savefig(imgLocation+".pdf",bbox_inches="tight",pad_inches=0.001);
        # ~ plt.show()
        plt.clf()
  
mainLoop(10)
# ~ mainLoop(19)

