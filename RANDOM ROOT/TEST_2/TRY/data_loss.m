function X = data_loss(X,min,max)
    n_ran = randi([min,max]);
    for m = 1 : n_ran
        random = randi([1,length(X)]);
        %Measured poses
        X(:,random) = X(:,random).*zeros(3,1);
    end
end