% propagacion de incertidumbre
% Pose del robot
hold off
n=100000;
mfi=atan();sg_fi=10;
mr=2;
fi=randn(n,1)*sg_fi*pi/180 +mfi;
% y=randn(n,1)*0.5-x;
r=mr+randn(n,1).*cos(fi-mfi)*0.1;
x=r.*cos(fi);
y=r.*sin(fi);
% y=-x;
figure(3)
plot(x,y,'.b');axis equal
xlim([0 15])
pose_r=[x,y];
mx=mr*cos(mfi);
my=mr*sin(mfi);
%%
% n=100;
% Medicion
x=randn(n,1)*.5;
y=randn(n,1)*0.1+2*x;
plot(x,y,'.b');axis equal
% pose_r=[x,y];
pose_z=[x,y]+[5,10];
%%
% clc
% Montecarlo
hold off
pp_=pp;
% pp_=pose_r;
pp=[];
N=1000;
nn=1000;
for i=1:N
    k=randperm(nn);
    p=pp_(k(1),:);
    k=randperm(n);
    p_=pose_r(k(1),:)+p;
    p_=get_new_pose()+p;
%     plot(p_(:,1),p_(:,2),'.b');hold on
    pp=[pp; p_];
%     pp_=p_;
end
% disp('listo')
%
% figure(1)
 hold on;plot(pp(:,1),pp(:,2),'.b')
%  plot(pp_(:,1),pp_(:,2),'.b');hold off
% plot(pose_r(:,1),pose_r(:,2),'.g');hold on
%  plot(pose_z(:,1)+mx,pose_z(:,2)+my,'.r');
% hold off
% xlim([0 30])
axis equal
disp('listo')
% figure(3)
% calculo de los segundos momentos:
M=cov([pp(:,1), pp(:,2)]);
det(M)


%%
% Calculo de histogrmas
% his=[pose_z];
figure(1)
% his=[pp];
his=[pp_sim+[0.05 0.05]];
% his=[pose_r];
discretizacion=[0.1,0.1];
% histogram2(pose_r(:,1),pose_r(:,2),'BinWidth',[0.1,0.1],'FaceColor','flat')
histogram2(his(:,1),his(:,2),'BinWidth',discretizacion,'DisplayStyle','tile','ShowEmptyBins','Off','Normalization','probability');axis equal
% histogram2(his(:,1),his(:,2),'BinWidth',discretizacion,'DisplayStyle','tile','ShowEmptyBins','Off');axis equal
hold on
% plot(mx,my,'xr')
plot(mean(his(:,1)),mean(his(:,2)),'xr')
hold off
colormap gray
axis equal
xlim([0 30])
% ylim([-2 25])
grid off
%%
% Calcular los valores propios y vectores propios de la matriz
CovMatrix = [10, 0; 0, 1];
CovMatrix=sigma_;
% CovMatrix=sigma_medicion;
% CovMatrix=sigma_robot;
[eigenvecs, eigenvals] = eig(CovMatrix);

% Escalar los vectores propios para que representen los semiejes de la elipse
scale_factor = 2*(max(diag(sigma_)))^0.5;  % Puedes ajustar este factor según tus necesidades

% Calcular los semiejes de la elipse
a = sqrt(eigenvals(1, 1)) * scale_factor;
b = sqrt(eigenvals(2, 2)) * scale_factor;

% Ángulo de rotación de la elipse en radianes
theta = atan2(eigenvecs(2, 1), eigenvecs(1, 1));

% Parámetro paramétrico t
t = linspace(0, 2*pi, 100);

% Coordenadas de la elipse
x = a * cos(t);
y = b * sin(t);

% Rotar la elipse
x_rotated = x * cos(theta) - y * sin(theta);
y_rotated = x * sin(theta) + y * cos(theta);

% Graficar la elipse
figure(2);
plot(x_rotated, y_rotated);
axis equal;
title('Elipse a partir de la matriz de varianza-covarianza');
xlabel('Eje X');
ylabel('Eje Y');
grid on;
%%
% Propagacion de incertidumbres usando aproximacion gaussiana
% Posicion del robot
syms w1 w2
fi=w1*10*pi/180 +atan(3/4);
r=10+w2.*cos(fi-mfi)*0.1;
j_=jacobian([r*cos(fi), r*sin(fi)],[w1;w2]);
sigma=[1 0;0 1];
w1_val=0
w2_val=0
J = double(subs(j_, [w1; w2], [w1_val; w2_val]));
sigma_robot=J*sigma*J'
%
x=w1*.5+mx;
y=w2*0.1+2*x+my;
j_=jacobian([x, y],[w1;w2]);
J = double(subs(j_, [w1; w2], [w1_val; w2_val]));
sigma_medicion=J*sigma*J'
%
% Sigma final:
x=w1+mx;
y=w2+my;
sigma_=sigma_robot+sigma_medicion

%%
% Superponer una elipse a los puntos
figure(1)
hold on
plot(x_rotated+mx+5, y_rotated+my+10,'-b','LineWidth',2);
hold off
xlim([0 30])
axis equal
%%
vee=[randn(n,1) randn(n,1)]*(sigma_)^0.5';
plot(vee(:,1),vee(:,2),'.b')
pp_sim=vee;
%%
function p=get_new_pose
mfi=0;sg_fi=10;
mr=2;
n=1;
fi=randn(n,1)*sg_fi*pi/180 +mfi;
% y=randn(n,1)*0.5-x;
r=mr+randn(n,1).*cos(fi-mfi)*0.1;
x=r.*cos(fi);
y=r.*sin(fi);
p= [x y];

end