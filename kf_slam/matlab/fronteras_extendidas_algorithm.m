% obstacle_map=zeros(600,600);
M_=sigma_map;
d1=diff(M_,1,1);d1=d1(:,1:599);
d2=diff(M_,1,2);d2=d2(1:599,:);
dd=(d1.^2+d2.^2).^0.5;
obss=obstacle_map(1:599,1:599);
idx=obss>0.3;
dd(idx)=0.0;
figure(1)
% imagesc(dd,[min(dd(:)) max(dd(:))]);colormap('jet');axis equal
imagesc(dd,[0.1 max(dd(:))]);colormap('jet');axis equal
figure(2)
imagesc(M_,[min(M_(:)) 0.6]);axis equal

%% Operacion con fronteras de incertidumbre
fronteras_elegibles=dd;
fronteras_elegibles(dd<0.1)=0;
imagesc(fronteras_elegibles)
c=[300 300];
pose=[0,100];
hold on 
locate=pose+c;
plot(locate(1),locate(2),'xr')

% find the closest frontier
idx_frontera=fronteras_elegibles;
idx_frontera(fronteras_elegibles>0)=1;
[x,y]=find(idx_frontera);X=[y x]; % frontier coords
dist=(X-locate).^2;dist_=(sum(dist,2)).^0.5;[~,idx]=min(dist_);
plot(X(idx(1),1),X(idx(1),2),'or')
hold off
axis equal
center=[X(idx(1),2) X(idx(1),1)];
% imshow(idx_frontera)
%% Prediccion de ganancia de info
phantom_map=M;
% imagesc(phantom_map)
fov_mask=zeros(600,600);
for i=1:600
    for k=1:600
        if (i-300)^2+(k-300)^2 <50^2
            fov_mask(i,k)=1;
        end
    end
end

fov_mask_=fov_mask(250:350,250:350);
M_masked=M(center(1)-50:center(1)+50,center(2)-50:center(2)+50);
mx=M(X(idx(1),2)+1,X(idx(1),1)+1);
% bayesian update
before=phantom_map(center(1)-50:center(1)+50,center(2)-50:center(2)+50);
phantom_map(center(1)-50:center(1)+50,center(2)-50:center(2)+50)=max(before,fov_mask_*mx+~fov_mask_.*M_masked);

disp('Divergence')
D=sum(sum(M.*log(M/alpha)))
disp('Divergence a priori')
D=sum(sum(phantom_map.*log(phantom_map/alpha)))
subplot(121)
imagesc(phantom_map);axis equal;colormap('jet')
subplot(122)
imagesc(M);axis equal;colormap('jet')




