n=100;
map_=zeros(n,3);
idx=0:1:n-1;

map_(:,1)=idx/n;
map_(:,3)=1-idx/n;
%%
idx=sigma_map~=sigma_map(1,1);
plasma_data = csvread('plasma_data.txt');
colormap(plasma_data)
imagesc(sigma_map,[min(sigma_map(idx)) sigma_mx]);axis equal;title('mapa linealizado')
% colormap(flipud(plasma_data))
% colormap(plasma_data(1:200,:))
colormap(plasma_data)
colorbar
%%
colorbar
figure(1)
imagesc(sigma_map(up_l(1):down_r(1),up_l(2):down_r(2))',[min(sigma_map(idx)) max(sigma_map(idx))]);axis equal;title('mapa linealizado')
colormap(map_)
figure(2)
imagesc(sigma_map(up_l(1):down_r(1),up_l(2):down_r(2)),[min(sigma_map(idx)) max(sigma_map(idx))]);axis equal;title('mapa linealizado')
colormap('winter')
figure(3)
imagesc(sigma_map(up_l(1):down_r(1),up_l(2):down_r(2)),[min(sigma_map(idx)) max(sigma_map(idx))]);axis equal;title('mapa linealizado')
colormap('copper')
%%
figure(4)
om3=om2(up_l(1):down_r(1),up_l(2):down_r(2))';axis equal
imshow(flipud(1-om3))
% imshow(om3)
%%
% max(sigma_map(idx))-0.7
sp=sigma_map(up_l(1):down_r(1),up_l(2):down_r(2))';
idx=sigma_map~=sigma_map(1,1);
imagesc(flipud(sp),[min(sigma_map(idx)) sigma_mx]);axis equal;title('mapa linealizado')
% imagesc(sigma_map(up_l(1):down_r(1),up_l(2):down_r(2))',[min(sigma_map(idx)) 0.3]);axis equal;title('mapa linealizado')
colormap(plasma_data)
