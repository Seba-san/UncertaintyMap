
% Este codigo es la implementacion de la simulacion con un modelo
% simplificado para 1D.
V0=0.1;r0=6;
% L1=0.1;L1x=25;

list_landmarks=[25 60;-1 -1];
Nl=size(list_landmarks,1);
f=5;
sigma_mx=0.6;
N=100;
M=ones(1,N)*sigma_mx;
rk=r0;t0=0;v=V0;
figure(1)
for i=1:N
    fov=rk-f:rk+f;
    for l=1:Nl
        if abs(rk-list_landmarks(1,l))<=f
            rval=r_val(v,i,t0);
            if list_landmarks(2,l)<0
                list_landmarks(2,l)=rval;
            end
            v_new=min([rval, list_landmarks(2,l)]);
            t0=i; v=v_new;
            rval=v_new;
            M=map_update(M,v,i,t0,fov);
        else
            M=map_update(M,v,i,t0,fov);
            rval=r_val(v,i,t0);
        end
    end
    plot(M,'.');hold on
    plot(rk,rval,'or');hold off
    rk_1=rk;
    rk=rk+1;
    pause(0.1)
end
disp('Fin')

function M=map_update(M,v,i,t0,fov)
rval=r_val(v,i,t0);
idx=M(fov)>rval;
M(fov(idx))=rval;
end

function v_=r_val(v,i,t0)
K=0.01;
v_=v+K*(i-t0);
end
%%
% 2D...






