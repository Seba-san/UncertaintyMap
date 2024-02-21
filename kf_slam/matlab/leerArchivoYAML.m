function contenido = leerArchivoYAML(nombreArchivo)
% funcion realizada por chatgpt
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
end