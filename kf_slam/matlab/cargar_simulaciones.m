% Patrón del nombre de archivo a buscar
cd /home/seba/Dropbox/1_Doctorado/exploration_method/kf_slam/matlab

% fecha_inicio = '20230727172301'; fecha_fin = '20230727180936'; %?
% fecha_inicio = '20230727131349'; fecha_fin = '20230727180936'; % UF simple
% fecha_inicio = '20230726194201'; fecha_fin = '20230727114040'; %simple
% fecha_inicio = '20230730165328'; fecha_fin = '20230730235036'; %double
% fecha_inicio = '20230730213921'; fecha_fin = '20230731015526'; %double
% fecha_inicio = '20230731131418'; fecha_fin = '20230731150121'; %double
fecha_inicio = '20230816184859'; fecha_fin = '20230816204929'; % simple 0.6
%
% Recordar que hay que cambiar los permisos de todos los .mat
k=0;
% patron = '2023072*.mat';
patron = '2023*.mat';


% Obtiene la lista de archivos que coinciden con el patrón
archivos = dir(patron);

% Número total de archivos encontrados
num_archivos = length(archivos);
Metricas=zeros(9,1);
% Comprobar si se encontraron archivos
figure(1)
if num_archivos == 0
    disp('No se encontraron archivos .mat con el patrón especificado.');A
else
    % Carga cada archivo .mat de forma automática
    for i = 1:num_archivos
       
        nombre_archivo = archivos(i).name;
        timestamp_str = nombre_archivo(1:14); % Extraer el timestamp del nombre del archivo
        timestamp_num = datenum(timestamp_str, 'yyyymmddHHMMSS'); % Convertir el timestamp a número de serie
        if timestamp_num >= datenum(fecha_inicio, 'yyyymmddHHMMSS') && ...
                timestamp_num <= datenum(fecha_fin, 'yyyymmddHHMMSS')
            fprintf('Cargando archivo: %s\n', nombre_archivo);
            
            load(nombre_archivo)
            %if exist('Data')
                save(nombre_archivo,'exp_map','land_map','states','P','obstacle_map','path_chassis','path_slam','Data') % Sobreescribo para hacer uso eficiente del espacio en dropbox
            %    clear Data
            %else
            %    save(nombre_archivo,'exp_map','land_map','states','P','obstacle_map','path_chassis','path_slam')
            %end
            Data.initial_pose
            Data.ending_condition
             k=k+1;
            subplot(3,4,k)
            [a,b,c,D,shannon,shannon_mean,land]=map_process(exp_map,Data.cell_size,Data.sigma_max,states,P);hold on
            T=path_chassis(end,3);
            K=1/0.1;B=300;
            %         plot(path_chassis(:,2)*K+300,-path_chassis(:,1)*K+300,'-g',path_slam(:,2)*K+300,-path_slam(:,1)*K+300,'-r');%axis equal;
            plot(path_chassis(:,2)*K+300,path_chassis(:,1)*K+300,'-g',path_slam(:,2)*K+300,path_slam(:,1)*K+300,'-r');%axis equal;
            hold off
            met=[a b c D shannon shannon_mean T land];
            Metricas(:,k)=met;
        end
        
    end
    disp('Carga automática de archivos .mat completada.');
end
    m=mean(Metricas(:,1:k)')
    std_=std(Metricas(:,1:k)')
    figure(2)
    boxplot(Metricas(:,1:k)')
    mm2=Metricas(:,1:k)';
    %%
    % tot=[m1' m2'];
    % bar(tot)
    k=9
    boxplot([mm1(:,k) mm2(:,k)])