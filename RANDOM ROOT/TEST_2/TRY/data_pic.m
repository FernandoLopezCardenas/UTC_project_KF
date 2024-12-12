function X = data_pic(X,num_pic,pic_min,pic_max)
    for m = 1 : num_pic
        n_ranx = randi([pic_min,pic_max]);
        n_rany = randi([pic_min,pic_max]);
        random = randi([1,length(X)]);
        %Measured poses
        X(:,random) = X(:,random).*zeros(3,1) + [n_ranx;n_rany;0];
    end
end