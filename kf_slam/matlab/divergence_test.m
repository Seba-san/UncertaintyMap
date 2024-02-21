x=-5:0.1:5;
N=length(x);
b=-0.717;
a=-0.416;
y=zeros(1,N);
for i=1:N
    if x(i)>0
         y(i)=1-(1/2)*exp(b*x(i)+a*x(i)^2);
    else
        k=-x(i);
        y(i)=(1/2)*exp(b*k+a*k^2);
    end
end

plot(x,y,'.')

%%
b=-0.717;a=-0.416;

step=0.1;
s=step/2:0.001:1;

sb=0.3;

x=step./(2*sb);
beta=1-exp(b*x+a*x.^2);
l_beta=log(beta/(1-beta));

x=step./(2*s);
ek=1-exp(b*x+a*x.^2);
l_ek=log(ek./(1-ek));

figure(1)
plot(s,l_ek,'b.',s,l_beta,'r.')
xlabel('\sigma')
title('log-odds exploration vs \sigma')
grid on

D=ek.*log(ek./beta);
figure(2)
plot(s(D>0),D(D>0),'.k',s(D<0),D(D<0),'.r')
title('Divergence vs \sigma')
xlabel('\sigma')
grid on
%%
b=-0.717;a=-0.416;

step=1;
aa=1e-1;
sigma=aa:aa/10:aa*10;
% s=step/2:step/10:step*2;

x=step./(2*sigma);
ek=1-exp(b*x+a*x.^2);
l_ek=log(ek./(1-ek));

figure(1)
plot(sigma,ek.^2,'b.')%,s,l_beta,'r.')
xlabel('\sigma');grid on;grid minor
title('log-odds exploration vs \sigma')






