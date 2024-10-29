% Leer todo el archivo CSV
data = readmatrix('bernoulli_old_PD.csv');

% Extraer la columna 19
columna_19 = data(:, 19); 

% Filtrar entradas que no sean NaN
columna_19 = columna_19(~isnan(columna_19)); 

% Mostrar el resultado
disp(columna_19);