#!/usr/bin/env python
"""
Copyright (c) 2023 Instituo de Automática

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
class PID:
    """
    Implementacion de un PID discreto en la forma:
    u=Kp(e + D + I)
    donde D es la parte derivativa discretizada mediante tustin (o bilineal) e I es la parte
    integral utilizando diferencias hacia atras.
    """
    def __init__(self):
        self.y=[0 for i in range(2)]
        self.y_sp=[0 for i in range(2)]
        self.I=0
        self.D=0
        self.limit=1
        self.u=0
        self.Normalizacion=1 

    def set_parameters(self,**kargs):
        """ Establece los parametros del PID
        
        Kp: Ganancia proporcional
        Ki: Ganancia integral
        Kd: Ganancia derivativa
        Ts: Periodo de muestreo
        N: Orden del filtro derivativo
        
        Tener en cuenta que Kp tambien se puede utilizar como constante para ajustar unidades. 
        Como el PID es de la forma Kp(e + D(Ki) + I(Ki)) resulta que Ki y Kd estan multiplicadas por Kp.
        """
        self.Kp=kargs.get('Kp')
        self.Ki=kargs.get('Ki')
        self.Kd=kargs.get('Kd')
        self.Ts=kargs.get('Ts')
        self.N=kargs.get('N',10)
        N=self.N
        Ts=self.Ts
        self.D_0=(N * Ts - 2.0) / (N * Ts + 2.0) # Bilinear or tustin
        self.D_1=2.0 / (N * Ts + 2.0)# Bilinear or tustin
        self.I_1=Ts / 2.0

    def run(self,y_sp_=float,y_=float)->float :
        """ Obtiene la señal de control a partir de la señal de error (set point - medida)  

        En caso de que el error requiera aplicar una función externa a esta clase, se debe
        llamar de la siguiente manera:
        error=funcion_externa()
        run(y_sp_=0,y_=-error)
              
        Inputs:
        .- y_sp: Set point
        .- y_: medida
        
        Outputs:
        .- u: señal de control (-self.limit, self.limit)
        """
        # Se acomodan las unidades
        y_=y_ * self.Normalizacion
        y_sp_=y_sp_ * self.Normalizacion
        self.y.insert(0,y_)
        self.y.pop()
        self.y_sp.insert(0,y_sp_)
        self.y_sp.pop()
        
        self.D = self.Kd * (self.D_1 * (self.y[0] - self.y[1])) - self.D_0 * self.D
        self.u = self.Kp * (self.y_sp[0] - self.y[0] + self.D)
        temp = self.Ki * self.I_1 * (self.y_sp[0]  - self.y[0] ) 
        # Antiwindup for integral part
        if abs(self.u+self.Kp*(temp+self.I))<self.limit: 
             self.u= self.u+self.Kp*(temp+self.I)
             self.I=self.I+temp
        else:
            self.u=abs(self.u)*self.limit/self.u 
            self.I=self.I-temp

        return self.u

if __name__=='__main__':
   pid=PID()
   pid.set_parameters(Kp=1.0,Ki=0.0,Kd=0.0,Ts=0.1)
   u=pid.run() # Control signal
