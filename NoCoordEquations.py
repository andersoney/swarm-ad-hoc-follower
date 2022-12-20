import numpy as np
import math
import matplotlib.pyplot as plt

class NoCoord(object):
  # Abstracts the estimated calculations about NoCoord algorithm.   
  
  ms1 = 4
  
  Tstart = staticmethod(lambda vmed,n: NoCoord.ms1/(vmed*(n+1)))
  Tarrive = staticmethod(lambda s,vmed,D: (D-s)/vmed)
  Tfilltarget = staticmethod(lambda s,vmed: s/vmed)
  Tleavecongestion = staticmethod(lambda s,vmed,D: (D-s)/vmed)
  
  f2 = staticmethod(lambda p,n,I,Ndist : p*(n**(1.5)/(1.5*math.sqrt(math.pi))-n)/Ndist)
  
  C = lambda s,vmed,n,D: NoCoord.Tstart(vmed,n) + NoCoord.Tarrive(s,vmed,D) + NoCoord.Tfilltarget(s,vmed) + NoCoord.Tleavecongestion(s,vmed,D)
  
  @staticmethod
  def bestK(f, xs, ys, I, s, vmed, D, Ndist, i0=0):
    A = sum([f(1,xs[i],I[i],Ndist)*(ys[i] - NoCoord.C(s,vmed[i],xs[i],D)) for i in range(i0,len(xs))])
    B = sum([f(1,xs[i],I[i],Ndist)**2 for i in range(i0,len(xs))])
    return A/B
  
  @staticmethod
  def calcTime(f, n, vmed, s, I, bestK, D, Ndist):
    Tfinal = f(bestK,n,I,Ndist)
    Tblock = [NoCoord.Tstart(vmed,n), NoCoord.Tarrive(s,vmed,D), NoCoord.Tfilltarget(s,vmed), Tfinal, NoCoord.Tleavecongestion(s,vmed,D)]
    return sum(Tblock)
  
  @staticmethod
  def checkKs(Y1, Y2, dataMean, a, option):
    # Code for confirming the best Ks and show the best difference between calcTimes.
    diff1 = sum([(Y1[i] - dataMean[i,a,option])**2 for i in range(len(dataMean[:,a,option]))])
    diff2 = sum([(Y2[i] - dataMean[i,a,option])**2 for i in range(len(dataMean[:,a,option]))])
    print("difference for chosen Ks:",diff1,diff2)
  
  @staticmethod
  def doTheFitting(xs,ys,maxIndex=None):
    maxFit = len(xs)//2 if maxIndex==None else maxIndex
    sqrtData = [math.sqrt(v) for v in xs]
    mymodel0 = np.poly1d(np.polyfit(sqrtData[:maxFit],ys[:maxFit],3))
    print(mymodel0)
    mymodel0values = mymodel0(sqrtData)
    return mymodel0values
  
  @staticmethod
  def plotEstimation(varValues, dataMean, a, option, s, D, Ndist):
    I = 0.5*np.array(dataMean[:,a,13])
    vmed = dataMean[:,a,16]

    K2 = NoCoord.bestK(NoCoord.f2,varValues,dataMean[:,a,option],I,s,vmed,D,Ndist)
    print("NoCoord K =", K2)
    Y2 = [NoCoord.calcTime(NoCoord.f2,varValues[i],vmed[i],s,I[i],K2,D,Ndist) for i in range(len(varValues))]
    plt.plot(varValues,Y2,label="Estimation"%K2)

    #Plot estimations by polynomial fitting.
    if False:
      mymodel0values = NoCoord.doTheFitting(varValues,dataMean[:,a,option],len(varValues))
      plt.plot(varValues,mymodel0values,label=algorithmsLabels[h][al][a]+" reg")
      mymodel1values = NoCoord.doTheFitting(varValues,Y1,len(varValues))
      plt.plot(varValues,mymodel1values,label="Est reg 1")
      mymodel2values = NoCoord.doTheFitting(varValues,Y2,len(varValues))
      plt.plot(varValues,mymodel2values,label="Est reg 2")
  
  @staticmethod
  def plotAndReturnEstimation(varValues, dataMean, a, option, s, D, Ndist, m, i_sf, Imed2, vmed2, plotIt=True):
    if 0 in dataMean[:,m,a,i_sf,13] and Imed2 != None:
      Imed = Imed2
    else:
      Imed = 0.5*np.array(dataMean[:,m,a,i_sf,13])
    if 0 in dataMean[:,m,a,i_sf,16] and vmed2 != None:
      vmed = vmed2
    else:
      vmed = dataMean[:,m,a,i_sf,16]

    K2 = NoCoord.bestK(NoCoord.f2,varValues,dataMean[:,m,a,i_sf,option],Imed,s,vmed,D,Ndist)
    # ~ print("NoCoord K =", K2)
    Y2 = [NoCoord.calcTime(NoCoord.f2,varValues[i],vmed[i],s,Imed[i],K2,D,Ndist) for i in range(len(varValues))]
    if plotIt: plt.plot(varValues,Y2,label="Estimation"%K2)

    return Y2
