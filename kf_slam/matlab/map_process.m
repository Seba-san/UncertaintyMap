function [a,b,c,D,shannon,shannon_mean,land]=map_process(exp_map,cell_size,sigma_mx,states,P)
% En esta funcion se calculan todos las metricas para estimar el
% performance de un sistema de exploracion.

s=cell_size; sig=sigma_mx; 
map_size=600;%x600

M=exp_map;
% figure(1)
% imagesc(M);axis equal
cov_=[sig^2 0;0 sig^2];
n_c=size(cov_,1);
limit=ones(1,n_c)*s/2;
beta=mvncdf(-limit,limit,[0,0],cov_);
% p=mvncdf([s/2 s/2;s/2 -s/2;-s/2 s/2;-s/2 -s/2],[0,0],[sig^2 0;0 sig^2]);
%  beta=p(1)-p(2)-p(3)+p(4); 

% b=0.717;a=0.416;
% c_=log(-M.^0.5 +1);
% a_=a*s^2/4;b_=b*s/2;
% sigma_map=((-b_+(b_^2-4*a_*c_).^0.5)./(2*a_)).^-1;
a_aprox=sigma_mx*beta.^(0.5); % Se asume que exp_map(1,1)=beta
sigma_map=a_aprox./exp_map.^(0.5);
% figure(2)
% imagesc(sigma_map,[min(sigma_map(:)) 0.3]);axis equal;title('mapa linealizado')
imagesc(sigma_map);axis equal;title('mapa linealizado')
colormap('jet');%colorbar;

% Calculo de divergencia del mapa
dkl=(cell_size^2)*(log(exp_map/beta)-1+beta./exp_map);
idx=exp_map<beta;
dkl(idx)=-dkl(idx);
dkl_map=sum(dkl(:));
% Calculo de la divergencia de los landmarks
Nl=(size(P,2)-2)/2;
cov_sigma_mx=cov_;
Dkl_landmarks=0;
for i=1:Nl
    id1=i*2+1;
    id2=i*2+2;
    cov_=P(id1:id2,id1:id2);
    det_=det(cov_sigma_mx)/det(cov_);
    dkl_li=0.5*(log(det_)-2+trace(cov_sigma_mx^(-1)*cov_));
    
    if det_<1
        dkl_li=-dkl_li;
    end
    Dkl_landmarks=dkl_li+Dkl_landmarks;
end

% Revisado hasta aca

alpha=2*beta;%-beta^2;
% disp('Divergence')
D=sum(sum(M.*log(M/alpha)));
% disp('Entropy')
dk=-M(M>alpha).*log(M(M>alpha));
shannon=sum(dk(:));
% disp('mean Entropy')
shannon_mean=mean(dk(:));

% Ensayo de algunas metricas
% Porcentaje de cobertura del FOV
% Real Pose
cell_size=0.1;
r_r=10/cell_size;
center_r=[31 31]./cell_size;

objetive=exp_map;
exploracion_minima=objetive(1,1);
% imagesc(objetive);axis equal;%colormap('jet')
% Generacion de mascara
ss_=size(objetive);
idx=zeros(ss_);
for i=1:ss_(1)
    for k=1:ss_(2)
        if (k-center_r(1))^2+(i-center_r(2))^2<r_r^2
            idx(i,k)=1;
        end
    end
end
idx=logical(idx);

% Test 1
fov_amount=objetive(idx)>exploracion_minima;
% disp('Porcentaje cobertura del FOV en torno al landmark')
a=(sum(fov_amount)*cell_size^2)*100/(pi*(r_r*cell_size)^2);
% histogram(objetive(idx))

% Test 2
% fov_amount=objetive(~idx)>exploracion_minima;
idx=zeros(ss_);
r_=0.5:0.01:300;
for j=1:size(r_,2)
    for i=1:ss_(1)
        for k=1:ss_(2)
            if (k-center_r(1))^2+(i-center_r(2))^2<(r_(j)/cell_size)^2
                idx(i,k)=1;
            end
        end
    end
    idx=logical(idx);
    fov_amount=objetive(idx)>exploracion_minima;
    extra_covered_area=(sum(fov_amount(:))*cell_size^2)*100./(pi*(r_(j).^2));
    if extra_covered_area <80
        break
    end
end
% idx=logical(idx);
% fov_amount=objetive(idx)>exploracion_minima;
%disp('cobertura extra por fuera del FOV')
%(sum(fov_amount)*cell_size^2)
% disp('Radio maximo alcanzado con cobertura mayor al 80%')
b=r_(j-1);

% extra_covered_area=((sum(fov_amount)*cell_size^2)*100./(pi*(r_extra.^2-(r_r*cell_size)^2)));
% extra_covered_area=((sum(fov_amount(:))*cell_size^2)*100./(pi*(r_extra.^2)));
% max(r_extra(extra_covered_area>80))

%Test 3
f_test=objetive>exploracion_minima;
d_max=0;
for i=1:ss_(1)
    for k=1:ss_(2)
        if f_test(i,k)>0.5 &&  (k-center_r(1))^2+(i-center_r(2))^2 >d_max
            d_max= (k-center_r(1))^2+(i-center_r(2))^2 ;
        end
    end
end
% disp('Distancia maxima alcanzada desde el landmark')
c=(d_max)^0.5*cell_size;
% disp('Pose landmark')
land=states(3:4);
 
end