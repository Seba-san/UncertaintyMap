#!/usr/bin python3
from k_means import k_means
import numpy as np


def silhouette_score(ass,cent,data):
    k=max(ass)# cantidad de clusters

    for i in range(k):
        idx=ass==i
        points_cluster=data[idx,:]
        np.sum(np.sqrt(np.sum((cent[0]-points_cluster)**2),1))#
        

    pass
    


if __name__=='__main__':   
    data=np.array([[0.1,0.2,0.3,10.1,10.2,10.3,-10.1,-10.2,-10.3,20.1,20.20,20.3,50.1,50.20,50.3]]).T
    K=5
    var_0=500
    var_1=500
    var_ant=0
    #ass,cent=k_means(data,3)
    for i in range(K):
        ass,cent=k_means(data,i+1)
        tot_var=0
        for k in range(i+1):
            idx=ass==k
            tot_var=np.var(data[idx,0])+tot_var # +np.var(data[idx,1]) variancia total
        
        if (var_0-var_1)>(var_1-tot_var) and i>1 and var_ant<=(var_0-var_1):        
            print('codo encontrado en ',i,'variacion maxima', var_0-var_1,'variacion siguiente', var_1-tot_var,'variacia actual',tot_var)
            import pdb; pdb.set_trace()
            var_ant=var_0-var_1
            var_0=var_1           
            var_1=tot_var                        
        else:
            #import pdb; pdb.set_trace()
            var_0=var_1
            var_1=tot_var
            


             
   
