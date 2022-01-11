function showFeatures(S, image)

    figure(2)
    imshow(image);
    hold on
    x = S.p(1,:);
    y = S.p(2,:);
    plot(x,y,'ys');

end