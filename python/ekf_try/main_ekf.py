import numpy as np
import scipy
import matplotlib.pyplot as plt
import random
# generate data
max_step=300
noise_true=10
v_true = 1
v=1
x_predicts=[]
x_predicts.append(0)
zs=[]
x_trues=[]
x_trues.append(1)
zs.append(0)
ts=[]
ts.append(0)
for i in range(1, max_step):
  zs.append(x_trues[i-1]*1.05+(random.random()-0.5)*noise_true)
  x_trues.append(x_trues[i-1]*1.05)
  x_predicts.append(v * i)
  ts.append(i)

#set model param
h=np.matrix([1,0])
a=np.matrix([[1,v],[0,1]])

#set noise
r=10
q=np.matrix([[0.01,0],[0,01.0]])

#set init
x_0=np.matrix([[0],[1]])
xs=[]
xs.append(x_0)
p_0=np.matrix([[1,0],[0,1]])
ps=[]
ps.append(p_0)
ks=[]
k_0=[[0],[0]]
ks.append(k_0)

#do ekf
for i in range(0,max_step):
    #predict
    if(i==max_step-1):
        aa=1
    x_c=a*xs[i]
    p_c = a * ps[i] * a.transpose()
    #update by obs
    hphr_inv = np.linalg.inv(h*p_c*h.transpose()+r)
    k_c=p_c*h.transpose()*hphr_inv
    x_c=x_c+k_c*(zs[i]-h*x_c)
    print((k_c * (zs[i] - h * x_c)).transpose())
    p_c=(np.identity(2)-k_c*h)*p_c

    ps.append(p_c)
    xs.append(x_c)
    ks.append(k_c)

#fetch result
res=[]
for i in range(0,max_step):
    res.append(xs[i][0,0])

#visulaization
plt.figure(figsize=(8,8))
plt.plot(ts, zs, color="red", marker=".", linewidth=0.0)
plt.plot(ts, x_trues, color="blue")
plt.plot(ts, x_predicts, color="yellow")
plt.plot(ts, res, color="black")
plt.show()
