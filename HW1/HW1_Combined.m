%% HW 1 - ME 6406 Machine Vision
% by Cody Houff
% 9/7/21

%% Problem 1
clear
% S = s/R
S = [-1:.1:1]; 



p = (1/pi).*(acos(S) - S.*sqrt(1-S.^2));
figure(1)
plot(S,p)

%% Problem 2a
clear

image_e = [122 121 115 113 118 122 125 127;
120 114 115 119 123 127 128 124;
115 116 122 124 127 128 124 121;
114 120 125 126 127 125 122 118;
118 124 125 125 123 121 117 117;
122 125 125 122 117 116 116 115];

values = unique(image_e); %creates a vector of the unique values

count_values = histcounts(image_e)'; % count of # unique 

figure(1)
bar(values,count_values)

cdf = cumsum(count_values); % the cumulative sum of the count of unique values 

qk = ((2^(8)-1)/(6*8))*cdf;

round_qk = round(qk);    
    
table = [values,count_values,cdf,qk,round_qk]

figure(2)
bar(round_qk,count_values)

image_e_eq = 0 * image_e;

for i = 1:length(values)
    image_e_eq(image_e==values(i)) = round_qk(i);
end
image_e_eq

%% Problem 2b
clear

old = imread("eyeball.png");

new = histeq(old);

figure(1)
imshowpair(old,new,'montage')

[y,x] = imhist(old);

[y1,x1] = imhist(new);

figure(2)
bar(x,y)

figure(3)
bar(x1,y1)

%% Problem 3a
clear
%5x5 matrix, omega = 2

%G(y,x)
%G(n,m)=(1/(2*pi*omega^2))^(-(m^2+n^2)/(2*omega^2))

%         n
%         ^
%         |
% w1  w2  w3  w4  w5   
% w6  w7  w8  w9  w10
% w11 w12 w13 w14 w15 -> m
% w16 w17 w18 w19 w20
% w21 w22 w23 w24 w25 


omega = 2;

for n = -2:1:2
    for m = -2:1:2

    G(n+3,m+3)=(1/(2*pi*omega^2))*exp(-(m^2+n^2)/(2*omega^2));
    
    end
end

G

%% Problem 3b
%  at pixel(X, Y)=(4, 5) 
% (1,1) is a the top left corner X axis points down and Y axis to the right 

z = [124 127 128 126 127 125 125 123 121]'; 

hx = [-1 -2 -1 0 0 0 1 2 1];
hy = [-1 0 1 -2 0 2 -1 0 1 ];

Gx = hx*z
Gy = hy*z

Gmag = sqrt(Gx^2+Gy^2)

Gdir_radians = atan2(Gy,Gx) %radians
 
 
%% Problem 3c
clear

image = imread("IC_pin.png");

[Gy,Gx] = imgradientxy(image);
[Gmag,Gdir] = imgradient(image);

figure(2)
imshowpair(Gx,Gy,'montage');
title('Gx and Gy'); 

figure(3)
imshowpair(image,Gmag,'montage');
title('orignial and Gmag');

%% Problem 3d
clear


image = imread("salt_and_pepper_checker.png");

for sigma = [1 2 5]
    mask_size = 2*3*sigma + 1;
    G = zeros (mask_size);

    for i = -floor(mask_size/2):1:floor(mask_size/2)
        for j = -floor(mask_size/2):1:floor(mask_size/2)
            G(i+ceil(mask_size/2), j+ceil(mask_size/2)) = (1/(2*pi*sigma^2))*exp(-(i^2 + j^2)/(2*sigma^2));
        end
    end
    image_blur = imfilter(image,G);
    figure()
    imshow(image_blur)
end

%% Problem 3e
clear

image = imread("checker.png");

sigma = 1;
for n = -3:1:3
    for m = -3:1:3
        B(n+4,m+4) = (1/(2*pi*sigma^2))*exp(-(m^2+n^2)/(2*sigma^2));
    end
end

% 13x13 matrix
sigma = 2;

for n = -6:1:6
    for m = -6:1:6
        A(n+7,m+7) = (1/(2*pi*sigma^2))*exp(-(m^2+n^2)/(2*sigma^2));
    end
end

Z = zeros(13);%create an 11x11 zero matrix load the 7x7 matrix

for n = -3:1:3
    for m = -3:1:3  
        Z(n+7,m+7) = B(n+4,m+4); % load 7x7 values into 13x13 zero matrix
    end
end


difference_of_gaussian = (Z-A); %difference of Gaussian

image_blur = imfilter(image,difference_of_gaussian);
imshow(image_blur);

%% Problem 4a
% 

image = imread('nut_and_shell.png');
bw_image = rgb2gray(image);
[y,x] = imhist(bw_image);
figure()
bar(x,y)
over = 100;
under = 20;
best = 50;

for threshold =[over under best]/255
    figure()
    binary_image = im2bw(bw_image, threshold);
    imshow(binary_image)
end


%% Problem 4b
clear

image = imread("nut_and_shell.png");
image_bw = rgb2gray(image);
threshold = 45/255;
image_binary = im2bw(image_bw, threshold);

figure()
image_rev = imcomplement(image_binary);

s = regionprops(image_rev, 'centroid');
a = regionprops(image_rev,'area');

centroids = cat(1,s.Centroid)
areas = cat(1,a.Area)

imshow(image_rev)
hold on
plot(centroids(:,1),centroids(:,2),'b*')
hold off
