import numpy as np
import math
import matplotlib.pyplot as plt

class SQF(object):
  ms1 = 4

  TEnterAlgArea = staticmethod(lambda vmed,n: (n*SQF.ms1)/(vmed*(n+1)))
  
  @staticmethod
  def TleavingTarget(vmed,s,D):
    L = math.sqrt(D**2 + s**2)
    ang1 = math.acos(L/(2*D))
    ang2 = math.atan(s/D)
    return (ang1 + ang2)*L/vmed
  
  TfillCorridor = staticmethod(lambda vmed,s,D: (D - s)/(vmed))
  
  @staticmethod
  def TLongestArrive(vmed,n,s,D): 
    a = 2*math.asin(s/D)
    Res = (math.pi - a/2 - (2*math.pi - a)/(n+1))*D/vmed 
    return Res
    
  
  C = staticmethod(lambda vmed,s,n,D: SQF.TfillCorridor(vmed,s,D) + SQF.TleavingTarget(vmed,s,D) + SQF.TEnterAlgArea(vmed,n) + SQF.TLongestArrive(vmed,n,s,D))
  
  @staticmethod
  def f1(K,n):
    return K*n
  
  @staticmethod
  def bestK(f, xs, ys, vs, s, D, i0=0):
    Cs = [SQF.C(vs[i],s,xs[i],D) for i in range(len(xs))]
    A = sum([f(1,xs[i])*(ys[i] - Cs[i]) for i in range(i0,len(xs))])
    B = sum([f(1,xs[i])**2 for i in range(i0,len(xs))])
    return A/B

  @staticmethod
  def doTheFitting(xs,ys,maxIndex=None):
    maxFit = len(xs)//2 if maxIndex==None else maxIndex
    mymodel0 = np.poly1d(np.polyfit(xs[:maxFit],ys[:maxFit],2))
    print(mymodel0)
    mymodel0values = mymodel0(xs)
    return mymodel0values

  @staticmethod
  def plotEstimation(varValues, dataMean, a, option, s, D):
    Imed = 0.5*np.array(dataMean[:,a,13])
    vmed = dataMean[:,a,16]
    K1 = SQF.bestK(SQF.f1,varValues,dataMean[:,a,option],vmed,s,D,0)
    print("SQF K =", K1)
    Y1 = [SQF.C(vmed[i],s,varValues[i],D) + SQF.f1(K1,varValues[i]) for i in range(len(varValues))]
    plt.plot(varValues,Y1,label="Estimation")
        
    # ~ Y1F = [SQF.f1(K1,varValues[i]) for i in range(len(varValues))]
    # ~ print("Est f")
    # ~ mymodel0values = SQF.doTheFitting(varValues,Y1F,len(varValues))
    # ~ plt.plot(varValues,mymodel0values,label="Est f")
    
    # ~ Y1C = [SQF.C(vmed[i],s,varValues[i],D) for i in range(len(varValues))]
    # ~ print("Est C")
    # ~ mymodelCvalues = SQF.doTheFitting(varValues,Y1C,len(varValues))
    # ~ plt.plot(varValues,mymodelCvalues,label="Est C")

    # ~ print("reg orig")
    # ~ mymodel1values = SQF.doTheFitting(varValues,dataMean[:,a,10],len(varValues))
    # ~ plt.plot(varValues,mymodel1values,label="reg orig")

  @staticmethod
  def plotAndReturnEstimation(varValues, dataMean, a, option, s, D, m, i_sf, Imed2, vmed2, plotIt=True):
    if 0 in dataMean[:,m,a,i_sf,13]:
      Imed = Imed2
    else:
      Imed = 0.5*np.array(dataMean[:,m,a,i_sf,13])
    if 0 in dataMean[:,m,a,i_sf,16]:
      vmed = vmed2
    else:
      vmed = dataMean[:,m,a,i_sf,16]
    K1 = SQF.bestK(SQF.f1,varValues,dataMean[:,m,a,i_sf,option],vmed,s,D,0)
    # ~ print("SQF K =", K1)
    Y1 = [SQF.C(vmed[i],s,varValues[i],D) + SQF.f1(K1,varValues[i]) for i in range(len(varValues))]
    if plotIt: plt.plot(varValues,Y1,label="Estimation")
    
    return Y1
