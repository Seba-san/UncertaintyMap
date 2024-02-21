% Este codigo procesa toda la informacion del galpon y la deja lista para
% ser almacenada y posprocesada adecuadamente.

cd /home/seba/Dropbox/1_Doctorado/exploration_method/kf_slam
addpath ./matlab/
plasma_data = csvread('plasma_data.txt');
colormap(plasma_data)
%%
experimento=load_map(Data,exp_map,plasma_data,obstacle_map,P,states)
%%
% para empezar el archivo
% experimentos={}
load('experimentos.mat')
n=size(experimentos,2);
experimentos{n+1}=experimento;
save('experimentos.mat','experimentos')
disp('guardado')
%%
% Juntar todos los datos en uno
% load('experimentos.mat')
n=size(experimentos,2);
idx=experimentos{1}{7};
acumulado=experimentos{1};
sigma_acumulado=[];
for i=2:n
    for k=1:6
        try
             acumulado{k}=[acumulado{k} experimentos{i}{k}];
        catch
             acumulado{k}=[acumulado{k}; experimentos{i}{k}];
        end
    end
end
disp('listo')
%%
subplot(141)
boxplot(acumulado{1},'OutlierSize', 1,'Symbol','.w')
ylabel('\sigma Map')
xlabel('N° experiment')
ylim([0 1.2])
subplot(142)
boxplot(acumulado{2},'OutlierSize', 1,'Symbol','.w')
ylabel('\sigma Landmarks')
xlabel('N° experiment')
ylim([0 1.2])
subplot(143)
% hold on
% yyaxis right
boxplot(acumulado{5},'OutlierSize', 1,'Symbol','.w')
ylabel('Relative entropy')
xlabel('N° experiment')
ylim([0 1400])
subplot(144)
boxplot(acumulado{6},'OutlierSize', 1,'Symbol','.w')
ylabel('Coverture')
xlabel('N° experiment')
ylim([0 600])
%%
% Muestran todas las muestras el mismo experimento
% load('experimentos.mat')
figure(3)
n=size(experimentos,2);
d_=[];
idx=[];
for i=1:n
    d_=[d_; experimentos{i}{1}'];
    nn=size(experimentos{i}{1},2);
    idx=[idx; i*ones(nn,1)];
end
subplot(121)
boxplot(d_,idx,'OutlierSize', 1,'Symbol','.w')
title('Uncertainty Map')
ylabel('\sigma cells')
ylim([0 1.2])
hold on
yyaxis right
for i=1:n
%     plot(i,experimentos{i}{5},'or') % divergencia
    plot(i,experimentos{i}{6},'xr') % cobertura
end
ylabel('Coverture')
xlabel('N° experiment')
ylim([0 600])
%
%sigma de los landmarks
figure(3)
n=size(experimentos,2);
d_=[];
idx=[];
k=2;
for i=1:n
    d_=[d_; experimentos{i}{k}'];
    nn=size(experimentos{i}{k}',1);
    idx=[idx; i*ones(nn,1)];
end

subplot(122)
boxplot(d_,idx,'OutlierSize', 1,'Symbol','.w')
title('Uncertainty Landmarks')
ylabel('\sigma landmarks')
ylim([0 1.2])
hold on
yyaxis right
for i=1:n
    plot(i,experimentos{i}{5},'or') % divergencia
%     plot(i,experimentos{i}{6},'xr') % cobertura
end
ylabel('Relative entropy')
xlabel('N° experiment')
ylim([0 1400])
%%
% Ploteo de las trayectorias
figure(1)
imagesc(sigma_map(up_l(1):down_r(1),up_l(2):down_r(2))');axis equal;title('\sigma_{map}')
cs=Data.cell_size;
cx=(down_r(1)-up_l(1))/2;
cy=(down_r(2)-up_l(2))/2;
hold on
plot(path_slam(:,1)./cs+cx,path_slam(:,2)./cs+cy,'-g');
% plot(path_slam(:,1)./cs+cx,path_slam(:,2)./cs+cy,'or');
hold off
%%
function experimento=load_map(Data,exp_map,plasma_data,obstacle_map,P,states)
% lee los datos y los ordena en una celda en "experimento"
close all
% ##### Linearizar el mapa
sigma_mx=Data.sigma_max;
cell_size=Data.cell_size;
M=exp_map;
cov_=[sigma_mx^2 0;0 sigma_mx^2];
N=size(cov_,1);
limit=ones(1,N)*cell_size/2;
beta=mvncdf(-limit,limit,[0,0],cov_);
a=sigma_mx*(beta^(1/N));
sigma_map=a./(M.^(1/N));
% imagesc(sigma_map)
figure(1)
[row, col] = find(sigma_map < sigma_mx-0.01);
up_l=[min(row)-2  min(col)-2];
down_r=[max(row)+2  max(col)+2];
figure(1)
imagesc(sigma_map(up_l(1):down_r(1),up_l(2):down_r(2))');axis equal;title('\sigma_{map}')
colormap(plasma_data)
% ##### Segmentacion de obstaculos
M_=sigma_map;
M_(M_>sigma_mx)=sigma_mx;
d1=diff(M_,1,1);d1=d1(:,1:599);
d2=diff(M_,1,2);d2=d2(1:599,:);
dd=(d1.^2+d2.^2).^0.5;
om=obstacle_map(2:end,2:end);
disp('computando sdf...')
sdf_map= signed_distance_function(om>0.5).*cell_size;
disp('fin sdf!')
dd(sdf_map<0.5)=0; % Elimina los obstaculos como fronteras
dd(om==0.5)=0;
disp('Computando...')
Th=0.3; % umbral
om2=om;
% om2(om>0.5)=1;
om2(sdf_map<0.2)=1;
om2(om<0.5)=0;
figure(2)
title('Obstaculos y fronteras')
imagesc(om2(up_l(1):down_r(1),up_l(2):down_r(2))');axis equal;title('Obstacle map')
colormap(plasma_data)
% imshow(om2')
% imagesc(sdf_map)
idx=dd>Th;
[a,b]=find(idx);
if size(a,1)~=0
    figure(2)
    imshow(om2')
    mx=max(max(dd));
    jet_=winter();
    hold on
    for i=1:size(a,1)
        val=dd(a(i),b(i));
        l=(val-Th)/(mx-Th);
        ii=fix(l*64);
        if ii==0
            ii=1;
        end
        %     jet_(ii,:)
        plot(a(i),b(i),'.','Color',jet_(ii,:))
        % pause()
    end
    % colormap jet;
    %  colorbar
    hold off
    axis equal    
    warning('Quedaron fronteras por descubrir')        
end
% ##### Calculo de estadisticos
% Sigmas
n=size(P,1);
sigmas=[];
for i=2:n/2
    p=P(i*2-1:i*2,i*2-1:i*2);
    sigmas=[sigmas det(p)^(1/4)];
end
% Landmarks
states_=reshape(states(3:end),2,[]);
n=size(states_,2);
% real_pose=[14.5 0; 14.5 8.5;14.5 -8.5; -14.5 8.5; -14.5 -8.5;...
%     -8 -8.5; -8 8.5; 0 -8.5; 0 8.5; 8 8.5]';
real_pose=[14.5 0; 14.5 8.5;14.5 -8.5; -14.5 8.5; -14.5 -8.5;...
    -8 -8.5; -8 8.5; 0 -8.5; 0 8.5; 8 8.5;8 -8.5]';
%real_pose=[14.5 8.5;14.5 -8.5;8 8.5;8 -8.5;0 8.5;0 -8.5;-8 8.5;-8 -8.5;-14.5 8.5;-14.5 -8.5;14.5 0]';
n2=size(real_pose,2);

err_=[];
sumados=0;
for i=1:n
    for k=1:n2
        err=states_(:,i)-real_pose(:,k);
        err=(sum(err.^2)).^0.5;
        if err<4 & sum(abs(real_pose(:,k)))~=0
            err_=[err_ err];
            sumados=sumados+1;
            states_(:,i)=[0 0]';
            real_pose(:,k)=[0 0]';
        end
    end
end
if n2~=n
    warning('Faltaron landmarks por descubrir')
    real_pose
end
% Cobertura
idx=obstacle_map<0.5;
coverture=sum(idx(:))*cell_size^2;
% Divergencia
[d_map,d_landmarks]=map_process_v2(Data,exp_map,states,P); 
divergence=d_map+d_landmarks;
[d_map,d_landmarks]=map_process_v2(Data,exp_map,states,P,1); % normalizado con sigma_max=1
divergence_fix=d_map+d_landmarks;

% Empaquetado de los datos
idx=sigma_map== sigma_map(1,1);
map_=sigma_map(~idx);
err_landmarks=err_;
sigmas_landmark=sigmas;
diver=divergence;
diver_fix=divergence_fix;
% Revisar que la normalizacion de la divergencia tenga sentido
% ## Identificacion
% sigma
if Data.sigma_max>0.7
    exp_number=0;
else
    exp_number=1;
end
% pose
if Data.initial_pose.y>0
    exp_number=exp_number+0;
else
    exp_number=exp_number+2;
end
% mapa
try
    if strcmp(Data.map,'galpon_0')
        % if strcmp(Data.map_size,'galpon_0')
        exp_number=exp_number+0;
    else
        exp_number=exp_number+4;
    end
catch
    if isfield(Data,'map') ||  isfield(Data,'map_size')
        exp_number=exp_number+0;
    end
end
exp_number=exp_number+1 % no me gusta que arranque en cero jaja. 1 (1- 1 arriba,2-0.6 arriba,3-1 abajo, 4-0.6 abajo)
experimento={map_',sigmas_landmark,err_landmarks,diver,diver_fix,coverture,exp_number}; % mapa, sigmas landmarks, err landmarks, diver, diver fix,covertura,experimento
end
