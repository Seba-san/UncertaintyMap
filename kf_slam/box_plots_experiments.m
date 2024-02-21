% todo en uno
%

sigma_mx=Data.sigma_max
cell_size=Data.cell_size;
M=exp_map;
% figure(1)
% imagesc(M);axis equal
% Maop process
[d_map,d_landmarks]=map_process_v2(Data,exp_map,states,P);
dtot=d_map+d_landmarks
[d_map,d_landmarks]=map_process_v2(Data,exp_map,states,P,1);
dtot_to_1=d_map+d_landmarks
% Linearizar el mapa
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
imagesc(sigma_map,[min(sigma_map(:)) sigma_mx]);axis equal;title('sigma_map')
s=cell_size;
map_size=600;%x600
x=s:s:map_size*s;
colorbar
colormap('jet')
%
% Entropia de shannon
beta=exp_map(1,1);
S=-0.5*log( (2*pi*exp(1) )^(N) *(a^(2*N)*exp_map(exp_map~=beta).^(-2) ) )*cell_size^2;
sum(S)
% Data
idx=sigma_map== sigma_map(1,1);
disp('maxima incertidumbre del mapa')
max(sigma_map(~idx))
disp('incertidumbre mediana del mapa')
median(sigma_map(~idx))
disp('incertidumbre promedio del mapa')
mean(sigma_map(~idx))


states_=reshape(states,2,[]);
n=size(states_,2);
% real_pose=[14 7;14 -7;7 7;7 -7;0 7;0 -7;-7 7;-7 -7;-14 7;-14 -7;14 0]';
real_pose=[14.5 8.5;14.5 -8.5;8 8.5;8 -8.5;0 8.5;0 -8.5;-8 8.5;-8 -8.5;-14.5 8.5;-14.5 -8.5;14.5 0]';
n2=size(real_pose,2);
err_=[];
sumados=0;
for i=1:n
    for k=1:n2
        err=states_(:,i)-real_pose(:,k);
        err=(sum(err.^2)).^0.5;
        if err<1 & sum(abs(real_pose(:,k)))~=0
            err_=[err_ err];
            sumados=sumados+1;
            states_(:,i)=[0 0]';
            
            real_pose(:,k)=[0 0]';
        end
    end
    
    
end
disp('erro mediano')
median(err_)
disp('erro promedio')
mean(err_)
disp('erro maximo')
max(err_)

disp('cobertura m2')
% idx=sigma_map<sigma_map(1,1);
idx=obstacle_map<0.5;
covertura=sum(idx(:))*cell_size^2

sumados


states_
real_pose
%
% Estimadores sobre la covarianza de los landmarks
n=size(P,1);
sigmas=[];
for i=2:n/2
    p=P(i*2-1:i*2,i*2-1:i*2);
    sigmas=[sigmas det(p)^(1/4)];
end
disp('land max')
max(sigmas)
disp('land medio')
median(sigmas)
disp('land promedio')
mean(sigmas)

%%
% Data ordenada
load('experimentos.mat')
idx=sigma_map== sigma_map(1,1);
map_=sigma_map(~idx);boxplot(map_)
err_landmarks=err_;
sigmas_landmark=sigmas;
diver=dtot;
diver_fix=dtot_to_1;
covertura;
experimento={map_,sigmas_landmark,err_landmarks,diver,diver_fix,covertura,[2 2]}; % mapa, sigmas landmarks, err landmarks, diver, diver fix,covertura,experimento
n=size(experimentos,2);
experimentos{n+1}=experimento;
save('experimentos.mat','experimentos')
%%
% mapa de exploracion
n=size(experimentos,2);
d_=[];
idx=[];
for i=1:n
    d_=[d_; experimentos{i}{1}];
    nn=size(experimentos{i}{1},1);
    idx=[idx; i*ones(nn,1)];
end
subplot(121)
boxplot(d_,idx,'OutlierSize', 1,'Symbol','.w')
title('mapa de exploracion')
ylabel('exploration map cels')
hold on
yyaxis right
for i=1:n
    plot(i,experimentos{i}{5},'or') % divergencia
%     plot(i,experimentos{i}{6},'xr') % cobertura
end
ylabel('Relative entropy')
xlabel('N° experiment')

%%
%sigma de los landmarks
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
title('sigmas landmarks')
ylabel('exploration map cels')
hold on
yyaxis right
for i=1:n
    plot(i,experimentos{i}{5},'or') % divergencia
%     plot(i,experimentos{i}{6},'xr') % cobertura
end
ylabel('Relative entropy')
xlabel('N° experiment')

%%
%error de los landmarks
n=size(experimentos,2);
d_=[];
idx=[];
k=3;
for i=1:n
    d_=[d_; experimentos{i}{k}'];
    nn=size(experimentos{i}{k}',1);
    idx=[idx; i*ones(nn,1)];
end

subplot(133)
boxplot(d_,idx,'OutlierSize', 1,'Symbol','.w')
title('error landmarks')

%% Recorte del mapa explorado

[row, col] = find(sigma_map < sigma_mx-0.01);
up_l=[min(row)-2  min(col)-2];
down_r=[max(row)+2  max(col)+2];
figure(1)
subplot(211)
imagesc(flipud(sigma_map(up_l(1):down_r(1),up_l(2):down_r(2))'));
axis off
% colorbar
axis equal
subplot(212)
om=obstacle_map(2:end,2:end);
om2=om;
om2(om>0.5)=1;
om2(om<0.5)=0;
axis equal

imshow(flipud(om2(up_l(1):down_r(1),up_l(2):down_r(2))'))