"""
En esta clase se implementa el filtro de kalman utilizado en el contexto del slam2D. 
"""

import numpy as np


class KF:
    def __init__(self,R_noise=0.1,Q_noise=0.1):
        """
        initialization system matrixs, states and covariance matrix.
        """
        self.A=np.array([[1,0],[0,1]],dtype='f') # system matrix
        #self.B=np.array([[1,1]],dtype='f').T # input matrix
        self.B=np.eye(2,dtype='f') # input matrix
        
        self.R_=np.array([[1,0],[0,1]],dtype='f').dot(R_noise) # Noise in measure
        self.Q_=np.array([[1,0],[0,1]],dtype='f').dot(Q_noise)  # Noise in states
        self.Q=self.Q_
        self.P=np.eye(2,dtype='f') # covariance matrix
        self.x=np.array([[0,0]],dtype='f')


        self.H=np.array([[1,1]],dtype='f') # measuere matrix
        self.Hs=-1 # 
        self.Hl=1
       
    def update_matrix(self,idx):
        """
        Update matrixs after measure maching.
        Update H,R
        Input:
            .- idx is a list of landmark mached
        Output:
            .- self.H updated
            .- self.R updated
        """
        
        k=idx.shape[0]
        idx=idx.T[0].tolist()
        n=self.x.shape[1]
        if n>4:
            pass
            #import pdb; pdb.set_trace() 

        
        # integrate new landmarks is left
        # H update
        self.H=np.zeros((2*k,n),dtype='f')
        l=0
        Hss=np.array([[self.Hs,0],[0, self.Hs]],dtype='f')
        Hll=np.array([[self.Hl,0],[0, self.Hl]],dtype='f')     
        # Ojo aca!! los idx no estan ordenadas de menor a mayor, no puede construir la matriz.
        # Hay que buscar otra forma de construirla.   
        for i in idx:
            i_=int(i+1)
            z1_array=np.r_[np.zeros((1,(i_-1)*2),dtype='f'), np.zeros((1,(i_-1)*2),dtype='f')]
            z2_array=np.r_[np.zeros((1,n-(i_+1)*2),dtype='f'), np.zeros((1,n-(i_+1)*2),dtype='f')]
            self.H[l:l+2]=np.c_[Hss,z1_array,Hll,z2_array]
            l=l+2
        # R update        
        self.R=np.zeros((2*k,2*k),dtype='f')
        l=0
        for i in idx:
            self.R[l:l+2,l:l+2]=self.R_
            l=l+2   

    def update(self,u,z): 
        """
        update states x (self.x) and covariance matrix (self.P)
        Inputs: 
            .- u: control action
            .- z: measures
        Outputs:
            .- self.x updated
            .- self.P updated
        """      
        #import pdb; pdb.set_trace() 
        # measure Z is in world reference, and z needs are in robot reference 
        loc=self.x[0][:2]
        z=(z.reshape(-1,2)-loc).reshape(-1,1)
        #
        self.x=self.x.T # For compatibility purposes
        n=self.x.shape[0]
        self.A=np.eye(n,dtype='f')
        #x_=np.matmul(self.A,self.x.T)+np.r_[np.matmul(self.B,u),np.zeros((n-2,1),dtype='f')]# prediction state
        x_=np.matmul(self.A,self.x)+np.matmul(self.B,u)# prediction state
        z_=np.matmul(self.H,self.x) # prediction measure
        P_=np.matmul(np.matmul(self.A,self.P),self.A.T)+self.Q # @ is equivalent to np.matmul, but @ is better for reading purposes 
        #P_=self.A@self.P@self.A.T+self.Q # @ is equivalent to np.matmul, but @ is better for reading purposes 
        S=np.matmul(np.matmul(self.H,P_),self.H.T)+self.R # previus step for reading purposes
        #S=self.H@P_@self.H.T+self.R # previus step for reading purposes
        K=np.matmul(np.matmul(P_,self.H.T),np.linalg.inv(S)) # kalman gain
        #K=P_@self.H.T@np.linalg.inv(S) # kalman gain
        self.x=x_+np.matmul(K,(z-z_)) # estimation
        #self.x=x_+K@(z-z_) # estimation
        P=P_-np.matmul(np.matmul(K,S),K.T) # covariance matrix
        #P=P_-K@S@K.T # covariance matrix
        self.P=np.dot(P,0.5)+np.dot(P.T,0.5)# for numerical stability
        self.x=self.x.T # For compatibility purposes

    def run(self):
        import pdb; pdb.set_trace()

        self.update_matrix([2,3])
        #self.add_new_landmark()
        self.update(np.array([[1,1]]).T,np.r_[np.array([[1,1]]).T,np.array([[1,1]]).T])
        import pdb; pdb.set_trace()

def main():
    # a typical case of use
    kf=KF(R_noise=0.1,Q_noise=0.1)
    kf.x=np.ones((8,1),dtype='f')
    kf.run()
    

if __name__=='__main__':
    main()