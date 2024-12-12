function gaussian_noise = noise(n,media,size,sigma)
    % Generar ruido gaussiano
    gaussian_noise = normrnd(media,sigma,size,n);
    % Limitar el ruido gaussiano al rango [-1, 1]
    gaussian_noise(gaussian_noise < -1) = -1;
    gaussian_noise(gaussian_noise > 1) = 1;

end