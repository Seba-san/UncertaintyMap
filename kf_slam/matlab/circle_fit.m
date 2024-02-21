% circle fit
ftest=test4;
imagesc(ftest);axis equal
%%
dtest=diff(ftest)+diff(ftest');
imagesc(abs(dtest))
axis equal
%%
objetive=abs(dtest);
% imagesc(objetive);axis equal
discrete=objetive>1e-2;
hold on
plot(center(1),center(2),'xr')
hold off

data=discrete;
ss_=size(objetive);

%%
% In this cell, a fitted circle is obtained
ftest=test5; % Exploration map
limite=ftest(300,300);
% limite=0.066;

dtest=diff(ftest)+diff(ftest');
objetive=abs(dtest); % Ajuste sobre las derivadas de lo explorado
% objetive=abs(ftest)>limite; % Ajuste sobre lo explorado solamente

cell_size=0.1; % cell size in metters
r=5/cell_size; center=[300,300]; % first point of search
x0=[center,r];
fun=@(x)fit_max_circle(x,objetive); % objetive function, max circle
% fun=@(x)fit_circle(x,objetive); % objetive function; best circle
%addpath ./matlab/
options = optimset('MaxFunEvals',2000,'MaxIter',2000);%,'Display','iter');
[xmin,gmin] = fminsearch(fun,x0,options); % Ojo que no minimiza bien, por ahi hay que hacerlo manualmente...
% imagesc(objetive>0);axis equal
imagesc(objetive);axis equal

hold on
plot(xmin(1),xmin(2),'xr')

x=xmin(1)-xmin(3):cell_size:xmin(1)+xmin(3);
y=xmin(2)+(xmin(3)^2-(x-xmin(1)).^2).^0.5;
y=[y xmin(2)-(xmin(3)^2-(x-xmin(1)).^2).^0.5];
plot([x x],y,'.r')
% Real pose:
r_r=10/cell_size;
center_r=[31 31]./cell_size;
x=center_r(1)-r_r:cell_size:center_r(1)+r_r;
y=center_r(2)+(r_r^2-(x-center_r(1)).^2).^0.5;
y=[y center_r(2)-(r_r^2-(x-center_r(1)).^2).^0.5];
plot([x x],y,'.g')
plot(center_r(1),center_r(2),'xg')

hold off
xmin*cell_size % circle fitted in metters (center, radious)

%%
cd /home/seba/Dropbox/1_Doctorado/exploration_method/kf_slam
addpath ./matlab/
load('matlab_matrix.mat')
%
beta= contenido{3}{2};
% map='one_landmark';
map='galpon';
plan=(contenido{1}{2}(2:end-1));
% plan='double';
% initial_point='-27321-2-v3';
initial_point=strcat(contenido{8}{2},'_',contenido{9}{2});
version='';
name=strcat('./matlab/' ,beta,'_',map,'_',plan,'_',initial_point,'_',version,'.mat');
save(name,'exp_map','land_map','states','P')
disp(name)
%%
% obtener divergencia
contenido=leerArchivoYAML('parameters.yaml');
sigma_mx=str2double(contenido{3}{2});
cell_size=str2double(contenido{5}{2});

% E=1-1./(1+exp(exp_map));
% L=1-1./(1+exp(land_map));
E=exp_map;
L=land_map;
M=E+L-E.*L;


s=cell_size; sig=sigma_mx;
p=mvncdf([s/2 s/2;s/2 -s/2;-s/2 s/2;-s/2 -s/2],[0,0],[sig^2 0;0 sig^2]);
 beta=p(1)-p(2)-p(3)+p(4);

alpha=2*beta;%-beta^2;
disp('Divergence')
D=sum(sum(M.*log(M/alpha)))
disp('Entropy')
dk=-M(M>alpha).*log(M(M>alpha));
shannon=sum(dk(:))
disp('mean Entropy')
mean(dk(:))

% Ensayo de algunas metricas
% Porcentaje de cobertura del FOV
% Real Pose
cell_size=0.1;
r_r=10/cell_size;
center_r=[31 31]./cell_size;

objetive=exp_map;
exploracion_minima=objetive(1,1);
imagesc(objetive);axis equal;%colormap('jet')
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
disp('Porcentaje cobertura del FOV en torno al landmark')
(sum(fov_amount)*cell_size^2)*100/(pi*(r_r*cell_size)^2)
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
disp('Radio maximo alcanzado con cobertura mayor al 80%')
r_(j-1)

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
disp('Distancia maxima alcanzada desde el landmark')
(d_max)^0.5*cell_size
disp('Pose landmark')
states(3:4)
%%
%Load special states
contenido=leerArchivoYAML('parameters.yaml');
%
beta= contenido{3}{2};
map='one_landmark';
plan=(contenido{1}{2}(2:end-1));
% plan='double';
% initial_point='-27321-2-v3';
initial_point=strcat(contenido{8}{2},'_',contenido{9}{2});
version='v5';
name=strcat('./matlab/' ,beta,'_',map,'_',plan,'_',initial_point,'_',version,'.mat');
load(name)
disp(name)
%%
px=4*cos(x*pi/180)+1;
py=4*sin(x*pi/180)+1;
px_=2*cos(x*pi/180+30*pi/180)+1;
py_=2*sin(x*pi/180+30*pi/180)+1;
plot(px,py,'.b',px_,py_,'.r')
