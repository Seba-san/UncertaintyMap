% Abre el archivo y lee su contenido como una cadena de texto
archivo = fopen('parameters.yaml', 'r');
contenido = fread(archivo, '*char')';
fclose(archivo);

% Busca el campo "initial_pose" en el contenido del archivo
inicio = strfind(contenido, 'initial_pose:');
fin = strfind(contenido, 'Q_noise:');
campo_initial_pose = contenido(inicio:fin-1);

% Extrae el valor de "x" del campo "initial_pose"
valor_x = str2double(regexp(campo_initial_pose, 'x: ([-\d\.]+) #m', 'tokens', 'once'));

% Extrae el valor de "y" del campo "initial_pose"
valor_y = str2double(regexp(campo_initial_pose, 'y: ([-\d\.]+) #m', 'tokens', 'once'));

% Muestra los valores de "x" y "y" de "initial_pose"
disp('Valores de initial_pose:');
disp(['x: ', num2str(valor_x), ' #m']);
disp(['y: ', num2str(valor_y), ' #m']);
%%
% Busca el campo "planner" en el contenido del archivo
inicio = strfind(contenido, 'planner:');
fin = strfind(contenido, 'distance_branch:');
campo_planner = contenido(inicio:fin-1);

% Extrae el valor del campo "planner"
valor_planner = extractBetween(campo_planner, "'", "'");

% Muestra el valor de "planner"
disp('Valor de planner:');
disp(valor_planner);
%%
% function leerParametros()
   % Abre el archivo y lee su contenido como una cadena de texto
archivo = fopen('parameters.yaml', 'r');
contenido = fread(archivo, '*char')';
fclose(archivo);

   
% Busca el campo "initial_pose" en el contenido del archivo
inicio = strfind(contenido, 'initial_pose:');
fin = strfind(contenido, 'Q_noise:');
campo_initial_pose = contenido(inicio:fin-1);

% Extrae el valor de "x" del campo "initial_pose"
valor_x = str2double(regexp(campo_initial_pose, 'x: ([-\d\.]+) #m', 'tokens', 'once'));

% Extrae el valor de "y" del campo "initial_pose"
valor_y = str2double(regexp(campo_initial_pose, 'y: ([-\d\.]+) #m', 'tokens', 'once'));

% Muestra los valores de "x" y "y" de "initial_pose"
disp('Valores de initial_pose:');
disp(['x: ', num2str(valor_x), ' #m']);
disp(['y: ', num2str(valor_y), ' #m']);
% Busca el campo "planner" en el contenido del archivo
inicio = strfind(contenido, 'planner:');
fin = strfind(contenido, 'distance_branch:');
campo_planner = contenido(inicio:fin-1);

% Extrae el valor del campo "planner"
valor_planner = extractBetween(campo_planner, "'", "'");

% Muestra el valor de "planner"
disp('Valor de planner:');
disp(valor_planner);
    
    % Busca el campo "sigma_max" en el contenido del archivo
    inicio_sigma_max = strfind(contenido, 'sigma_max:');
    fin_sigma_max = strfind(contenido, 'fov:', inicio_sigma_max);
    campo_sigma_max = contenido(inicio_sigma_max:fin_sigma_max-1);

    % Extrae el valor del campo "sigma_max"
    valor_sigma_max = str2double(regexp(campo_sigma_max, 'sigma_max: ([-\d\.]+) #m', 'tokens', 'once'));

    % Muestra el valor de "sigma_max"
    disp('Valor de sigma_max:');
    disp(valor_sigma_max);
% end
%%
% Abre el archivo y lee su contenido como una cadena de texto
nombreArchivo='parameters.yaml';
   % Abre el archivo y lee su contenido como una cadena de texto
    archivo = fopen(nombreArchivo, 'r');
    contenidoTexto = fread(archivo, '*char')';
    fclose(archivo);

    % Divide el contenido en líneas
    lineas = strsplit(contenidoTexto, '\n');

    % Elimina las líneas de comentarios y vacías
    lineas = lineas(~startsWith(lineas, '#') & ~cellfun('isempty', lineas));

    % Inicializa las celdas para almacenar el contenido
    contenido = cell(length(lineas), 1);

    % Recorre cada línea y guarda el contenido en celdas
    for i = 1:length(lineas)
        linea = lineas{i};

        % Busca y elimina los comentarios en la línea
        indiceComentario = strfind(linea, '#');
        if ~isempty(indiceComentario)
            linea = linea(1:indiceComentario(1)-1);
        end

        % Divide la línea en clave y valor si no está vacía
        if ~isempty(linea)
            [clave, valor] = strtok(linea, ':');

            % Elimina los espacios en blanco alrededor de la clave y valor
            clave = strtrim(clave);
            valor = strtrim(valor(2:end)); % Se elimina el ":" al inicio del valor

            % Almacena la clave y valor en la celda correspondiente
            contenido{i} = {clave, valor};
        end
    end

    % Elimina las celdas vacías generadas por líneas de comentario o vacías
    contenido = contenido(~cellfun('isempty', contenido));
