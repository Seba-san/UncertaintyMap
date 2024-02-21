cd /media/seba/Respaldo/seba/Doctorado/investigacion/simulacion/ws/src/pioneer2dx/scripts/tmp
%%
dict=csvread('dict.csv');
%%
figure(1)
plot(dict(1,:),dict(2,:),'.')
%plot(dict(2,1:end-1),dict(1,1:end-1))
polyfit(dict(1,1:end-2),dict(2,1:end-2),1)