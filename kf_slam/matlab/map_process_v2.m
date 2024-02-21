function [D_map,Dkl_landmarks]=map_process_v2(Data,exploration_map,states,P,sigma_mx)
if nargin~=5
    sigma_mx=Data.sigma_max;
end

M=exploration_map;
cell_size=Data.cell_size;
% obtiene beta
cov_=[sigma_mx^2 0;0 sigma_mx^2];
N=size(cov_,1);
limit=ones(1,N)*cell_size/2;
beta=mvncdf(-limit,limit,[0,0],cov_);
% obtiene la constante a
a=sigma_mx*(beta^(1/N));
% linealiza todo el mapa
sigma_map=a./(M.^(1/N));
if nargin==5    
    M=M(M~=M(1,1)); % para cuando cambias el beta
end

% obtiene entropia relativa del mapa:
D=cell_size^2*(log(M/beta)-N/2+(N/2)*beta^(2/N)./(M.^(2/N)));
D(M<beta)=-D(M<beta);
D_map=sum(D(:));
% entropia relativa de los landmarks
n=(size(states,2)-2)/2; % Cantidad de landmarks
cov_sigma_mx=cov_;
Dkl_landmarks=0;
for i=1:n
    id1=i*2+1;
    id2=i*2+2;
    cov_l=P(id1:id2,id1:id2);
    det_=det(cov_sigma_mx)/det(cov_l);
    dkl_li=0.5*(log(det_)-N+trace(cov_sigma_mx^(-1)*cov_l));
    if det_<1
        dkl_li=-dkl_li;
    end
    Dkl_landmarks=dkl_li+Dkl_landmarks;
end

% disp('Divergencia del mapa')
% D_map
% Dkl_landmarks
end