function distance_field = signed_distance_function(matrix)
% Calcula la función de distancia con signo para una matriz de obstáculos

% Obtener dimensiones de la matriz
[rows, cols] = size(matrix);

% Inicializar matriz de la función de distancia con signo
distance_field = inf(rows, cols);
[m,n]=find(matrix);
% Recorrer la matriz para calcular distancias
for k=1:size(m,1)
    for i = 1:rows
        for j = 1:cols
            % Si el punto es un obstáculo, asignar distancia cero
            d=sqrt((m(k)-i)^2+(n(k)-j)^2);
            if distance_field(i,j)>d
                distance_field(i,j)=d;
            end
            
        end
    end
end
end
