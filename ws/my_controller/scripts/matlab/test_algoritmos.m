cd /media/seba/Respaldo/seba/Doctorado/investigacion/simulacion/ws/src/my_controller/scripts/matlab
%%
Ts=0.01;
Tf=10;
t=0:Ts:Tf;
x_in=zeros(1,length(t));
T_ini=2;
x_in(t>T_ini)=1;
% x_in=t;
T_ini=6;
x_in(t>T_ini)=0;

plot(t,x_in,'b.')
%%
% Calculo de la derivada
N=0;
D0=(N*Ts-2)/(N*Ts+2);
D1=2/(N*Ts+2);
I1=Ts/2;
 D0=0;
 D1=2/Ts;


L=length(x_in);
y=zeros(1,L);
yp=zeros(1,L);
for i=2:L
%     y(i)=D1*(x_in(i)-x_in(i-1))+D0*y(i-1);
%      y(i)=D1*(x_in(i)-x_in(i-1))+D0*y(i-1);
     y(i)=I1*x_in(i)+y(i-1);
     yp(i)=D1*(y(i)-y(i-1))+D0*yp(i-1);
end
% plot(t,x_in,'k.');hold on
plot(t,y,'k.');hold on
plot(t,yp,'b.');hold off