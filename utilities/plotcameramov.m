function t_n = plotcameramov(t,img, keypoints_img, t_prec,i)

%t_n = abs(t_prec) + abs(t);
figure(3)
[x;y;z] = t*[0 0 0];
subplot(1,2)
plot()
% 
% plot(i, t_n(1), 'r.'); hold on
% axis tight
% subplot(2,3,2)
% plot(i, t_n(2), 'r.'); hold on
% subplot(2,3,3)
% plot(i, t_n(3), 'r.'); hold on

x_keypoints = keypoints_img(:,1);
y_keypoints = keypoints_img(:,2);

subplot(2,2)
imshow(img,[]); hold on
plot(x_keypoints,y_keypoints,'x');
hold off
