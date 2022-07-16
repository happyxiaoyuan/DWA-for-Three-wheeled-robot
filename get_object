#coding='gbk'
import matplotlib.pyplot as plt
import math
import numpy as np

a=np.loadtxt('F:\python_work\map.tex')
ob=[]

a=np.array(a.astype(int))
print(a)
for i in range(a.shape[0]):
    for j in range(a.shape[1]):
        if(a[i][j]<55):
            ob.append([j*0.1-4,(80-1-i)*0.1-4])

ob = np.array(ob)

np.savetxt('F:\python_work\ob.tex',ob)
#plt.imshow(a,cmap='gray')
plt.scatter(ob[:,0],ob[:,1])
plt.show()
print(ob)
