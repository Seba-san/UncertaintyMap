function err=fit_circle(x,objetive)
% Se calcula el error del ajuste, ajustando un circulo y ponderandolo por
% la funcion objetivo. Se ajustara donde esten los maximos.
center=[x(1),x(2)];
r=x(3);
ss_=size(objetive);
err=0;
for i=1:ss_(1)
    for k=1:ss_(2)
        err=abs(objetive(i,k)*((i-center(1))^2+(k-center(2))^2-r^2))+err;      
    end
end
end