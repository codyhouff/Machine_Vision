%% HW 4 - ME 6406 Machine Vision
% by Cody Houff
% 11/17/21

%% Prob 1a: Pose Estimation [1]
% Hough Transform
% 9 straight edges in ‘block.png’.
% Find the 9 normal vectors n=(af,bf,c)^T/|(af,bf,c)^T|
clc
clear
close all

image = imread('block.png');

f = .79;
Dx = 6*10^-4;
Dy = 6*10^-4;
Cx = 376;
Cy = 240; 


[h,T,R] = hough(image);
imshow(h,[],'XData',T,'YData',R,'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho');
axis on, axis normal, hold on;

P  = houghpeaks(h,9,'threshold',ceil(0.3*max(h(:))));
x = T(P(:,2)); y = R(P(:,1));
plot(x,y,'s','color','white');

lines = houghlines(image,T,R,P,'FillGap',9,'MinLength',7);
figure(), imshow(image), hold on
max_len = 0;
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   
   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2, 'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
   
   text((xy(2,1)+xy(1,1))/2,(xy(2,2)+xy(1,2))/2, append('L:',strcat(int2str(k))), 'Color', 'red');
   text(xy(1,1),xy(1,2), append('a:',strcat(int2str(k))), 'Color', 'yellow');
   text(xy(2,1),xy(2,2), append('b:',strcat(int2str(k))), 'Color', 'red');
   
   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end

z = 1;
for s = 1:length(lines)
    
 u1 = lines(s).point1(1);
 v1 = lines(s).point1(2);
 
 u2 = lines(s).point2(1);
 v2 = lines(s).point2(2);

 ud1 = (u1-Cx)*Dx;
 ud2 = (u2-Cx)*Dx;
 vd1 = (v1-Cy)*Dy;
 vd2 = (v2-Cy)*Dy;

    m = (vd2-vd1)/(ud2-ud1);

    b = vd1-m*ud1;

    % -m*x+y-b = 0;

    A = -m;
    B = 1;
    C = -b;
    
    
    n = [A*f;B*f;C]/norm([A*f;B*f;C]);
    a = n(1);
    b = n(2);
    c = n(3);
    
    n_array_abc(s,:) = [A*f;B*f;C]/norm([A*f;B*f;C]);

                                             %-1 0 0
    N = [-1 0 0; -1 0 0; 0 0 -1; -1 0 0; 0 0 -1; 0 0 -1; 0 -1 0; 0 -1 0; 0 -1 0];
    P = [10 0 0; 10 0 10; 0 0 10; 10 10 10; 10 0 10; 0 10 10; 0 10 10; 0 10 0; 10 10 10];
    
%     A*f.*N(s,:)
%     B*f.*N(s,:)
%     C.*N(s,1:2)
%     A.*N(s,3)
%     H(z,:) = [1,3,44,534,43,534,3455]

    
    H(z,:) =   [a.*N(s,:) b.*N(s,:) c.*N(s,1:2) c.*N(s,3) 0 0 0];
    H(z+1,:) = [a.*P(s,:) b.*P(s,:) c.*P(s,1:2) c.*P(s,3) a b c];
    z = z+2;
   
end




add = 19;

Nc = [-0.4226, -0.6943, -0.5826];
No = [0, 0, 1];

%add these 6 eqs from research paper eq# 20, and #27
% useing No and Nc
H(add,:) =   [No(1) No(2) No(3) 0 0 0 0 0 0 0 0 0];
H(add+1,:) = [0 0 0 No(1) No(2) No(3) 0 0 0 0 0 0];
H(add+2,:) = [0 0 0 0 0 0 No(1) No(2) No(3) 0 0 0]; 

H(add+3,:) = [Nc(1) 0 0 Nc(2) 0 0 Nc(3) 0 0 0 0 0];
H(add+4,:) = [0 Nc(1) 0 0 Nc(2) 0 0 Nc(3) 0 0 0 0];
H(add+5,:) = [0 0 Nc(1) 0 0 Nc(2) 0 0 Nc(3) 0 0 0];


H;
b_matrix = [zeros(1,18),Nc(1),Nc(2),Nc(3),No(1),No(2),No(3)]';


n_array_abc

%% Prob 1b: Pose Estimation [1]
% [R] and T

V = linsolve(H,b_matrix)


r_matrix = [V(1:3)';
            V(4:6)';
            V(7:9)']
        
T_array = [V(10:12)]
'check Tz ~ 120'
T_array(3)

'check inv(T) ~ transpose(T)'
inverse_r = pinv(r_matrix)

transpose_r = transpose(r_matrix)





%% Problem 2a: Artificial Neural Network 
% Design an ANN to recognize O and T on a binary 4x4 square grid.

%Define your ANN structure by specifying the number of inputs, 
%hidden layers, layer nodes, and outputs. Provide a schematic of your 
%chosen ANN structure.

clear
clc
close all

% solved on attached paper and commented in code

%TrainingData is the input: 16 inputs, 0 or 1
%  K is input layer:        16 nodes
%    w_pk is K-P weights:   (16x4 matrix)
%  P is second layer:       4 nodes
%    w_qp is P-Q weights:   (4x2 matrix)
%  Q is output layer:       2 nodes
%O_q is the output:         2 outputs, [O;T] confidence from 0 to 1

%% Problem 2b: Artificial Neural Network 
%Derive the weight update rule for your ANN, assuming a unipolar 
%sigmoid function for each processing element.

% solved on attached paper 

%% Problem 2c: Artificial Neural Network 
%Train with the data in Fig 2(a). Show the convergence curve 
%(mean squared error vs. number of epoch). 
%Save the weights of nodes in a “.mat” file




%TrainingData is the input: 16 inputs, 0 or 1
%  K is input layer:        16 nodes
%    w_pk is K-P weights:   (16x4 matrix)
%  P is second layer:       4 nodes
%    w_qp is P-Q weights:   (4x2 matrix)
%  Q is output layer:       2 nodes
%O_q is the output:         2 outputs, [O;T] confidence from 0 to 1 

K_nodes = 16;
P_nodes = 4;
Q_nodes = 2;

%random numbers from [-1 to 1]
%w_pk = (randi([0 200],K_nodes,P_nodes)-100)./100 %weights of K to P layer
%w_qp = (randi([0 200],P_nodes,Q_nodes)-100)./100 %weights of P to Q layer

%save('Initial_weights2.mat','w_pk','w_qp');
load('Initial_weights2.mat');

%training data
%O Set
O1 = [0 1 1 0; 1 0 0 1; 1 0 0 1; 0 1 1 0];
O2 = [1 1 1 0; 1 0 1 0; 1 0 1 0; 1 1 1 0];
O3 = [0 1 1 1; 0 1 0 1; 0 1 0 1; 0 1 1 1];
O4 = [1 1 1 0; 1 0 0 1; 1 0 0 1; 1 1 1 0];

%T Set
T1 = [0 1 0 0; 1 1 1 1; 0 1 0 0; 0 1 0 0];
T2 = [0 0 1 0; 1 1 1 1; 0 0 1 0; 0 0 1 0];
T3 = [1 1 1 1; 0 1 0 0; 0 1 0 0; 0 1 0 0];
T4 = [1 1 1 1; 0 0 1 0; 0 0 1 0; 0 0 1 0];


Train_Data = [O1(:) O2(:) O3(:) O4(:) T1(:) T2(:) T3(:) T4(:)];

% Step 0 (Initialization)

mu = .1;
error_max = .05; % recommended .05
error_comb = error_max+1; % total error
error_matrix = [];
error_sum = 0;

num_epochs = 0;

while error_comb > error_max

    % Step 1 (Training loop)
    for k = 1:8 % train through four O's (add in T's later)

        if k<=4 % Letter O
            desired = [1;0];    
        else %k>=5 Letter T 
            desired = [0;1];
        end
        
        %Step 2 (Forward propagation)
        %Step 3 (Output error measure)
        [error_comb,O_p,O_q] = forward(Train_Data(:,k),w_pk ,w_qp,desired);
      
        
        % Step 4 (Error back propagation) 
        % For q->p:
        d_q = hprime(O_q).*(desired - O_q); % 2x1 matrix
        change_wqp = (mu.*d_q*O_p)'; % (2x1 * 1x4)' =  4x2 matrix
        w_qp = w_qp + change_wqp; %calc w_qp new = 4x2 matrix
        
        % For p->k:
        d_p = hprime(O_p).*sum(w_qp*d_q); % 1x8 matrix
        change_wpk = (mu.*d_p'*Train_Data(:,k)')'; % (4x1 * 1x16)' = 16x4 matrix
        w_pk = w_pk + change_wpk; %calc w_pk new = 16x4 matrix
        
        error_sum = error_comb + error_sum;
        
    end
    
    error_comb = error_sum;
    % 8 trainings done
    % update epoch count and error matrix
    
    error_matrix = [error_matrix; error_comb];
    num_epochs = num_epochs + 1;
    mu = sqrt(error_comb);
    
    error_sum = 0;   
end

save('adjusted_weights.mat','w_pk','w_qp');
plot(1:size(error_matrix),error_matrix,'b.')
title('Error vs Number of Epochs')
xlabel('Number of Epochs');
ylabel('Combined Error');

%% Problem 2c: Artificial Neural Network 
% Test the data in Fig 2(b) by reading the weights of nodes 
% in your “.mat” file. Show the output values and results. 

%Test Data
T1 = [1 1 1 1; 1 0 0 1; 1 0 0 1; 1 1 1 1];
T2 = [1 1 1 1; 0 1 1 0; 0 1 1 0; 0 1 1 0];
T3 = [1 1 1 1; 0 0 0 0; 0 0 0 0; 0 1 1 0];

Test_Data = [T1(:), T2(:), T3(:)];

 for k = 1:3 % test the trained NN on test data     
    [error_comb,O_p,O_q] = forward(Test_Data(:,k),w_pk ,w_qp,[0;0]);          
    disp(['test # ',num2str(k)])
    disp(['O ',num2str(O_q(1)),' confidence'])
    disp(['T ',num2str(O_q(2)),' confidence'])
    disp([' ']) 
 end
 
 



%% Prob 3 part 1a:  Artificial Color Contrast (ACC)
%  Derive the following equations with fk(x,y) = +(R-G) and +(R+G-B)
clc
clear
close all

% derived on attached paper

%% Prob 3 part 1b:  Artificial Color Contrast (ACC)
%  Perform the ACC transformation (σc=1, σs=10) on sample 
% color patterns (100×100 each) with the following combinations: 
% 1-2-3, 1-2-c, 1-b-3, 1-b-c, a-2-3, a-2-c, a-b-3, a-b-c

target_test1 = [170,110,108]; % test on class target
noise_test1 = [169,90,80]; % test on class noise

target1 = [225,88,96]; %hw prob target
noise1 = [149,135,134]; %hw prob noise

n=2;
if (n==1)
    target1 = target_test1;
    noise1 = noise_test1;  
end
if (n>1)
    target1 = [225,88,96];
    noise1 = [149,135,134];  
end

[img1,target1,noise1,dist1] = img_start(target1(1),target1(2),target1(3),noise1(1),noise1(2),noise1(3));

figure()
for m = 1:2:16
    subplot(8,2,m);
    imshow(img1);
    %title(['dist: ' num2str(round(dist1))]);
end

img_double = im2double(img1)*255;

img_red = img_double(:,:,1);
img_green = img_double(:,:,2);
img_blue = img_double(:,:,3);

sigma_c =  1;
sigma_s = 10;
hsize_c = 2*3*sigma_c+1;
hsize_s = 2*3*sigma_s+1;

Gc = fspecial('gaussian', hsize_c, sigma_c);
Gs = fspecial('gaussian', hsize_s, sigma_s);
Gc_big = fspecial('gaussian', hsize_s, sigma_c);


h1 = imfilter(img_red,Gc_big-Gs)+imfilter(img_green,Gs);
h2 = imfilter(img_green,Gc_big-Gs)+ imfilter(2*img_green-img_red,Gs);
h3 = imfilter(img_blue,Gc_big-Gs)+imfilter(img_blue-img_red+img_green,Gs);
ha = imfilter(img_red,Gc_big-Gs)+imfilter(img_blue-img_green,Gs);
hb = imfilter(img_green,Gc_big-Gs)+imfilter(img_blue-img_red,Gs);
hc = imfilter(img_blue,Gc_big-Gs)+imfilter(2*img_blue-img_red-img_green,Gs);

k=2;
m=2;

[img,target2,noise2,dist2] = img_end(h1,h2,h3);
subplot(8,2,k);imshow(img); k=k+2; %title(['dist: ' num2str(round(dist2))]);

[img,target2,noise2,dist2] = img_end(h1,h2,hc);
subplot(8,2,k);imshow(img); k=k+2; %title(['dist: ' num2str(round(dist2))]);

[img,target2,noise2,dist2] = img_end(h1,hb,h3);
subplot(8,2,k);imshow(img); k=k+2; %title(['dist: ' num2str(round(dist2))]);

[img,target2,noise2,dist2] = img_end(h1,hb,hc);
subplot(8,2,k);imshow(img); k=k+2; %title(['dist: ' num2str(round(dist2))]);

[img,target2,noise2,dist2] = img_end(ha,h2,h3);
subplot(8,2,k);imshow(img); k=k+2; %title(['dist: ' num2str(round(dist2))]);

[img,target2,noise2,dist2] = img_end(ha,h2,hc);
subplot(8,2,k);imshow(img); k=k+2; %title(['dist: ' num2str(round(dist2))]);

[img,target2,noise2,dist2] = img_end(ha,hb,h3);
subplot(8,2,k);imshow(img); k=k+2; %title(['dist: ' num2str(round(dist2))]);

[img,target2,noise2,dist2] = img_end(ha,hb,hc);
subplot(8,2,k);imshow(img); k=k+2; %title(['dist: ' num2str(round(dist2))]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

k=1;
figure()
[img1,target1,noise1,dist1] = img_start(target1(1),target1(2),target1(3),noise1(1),noise1(2),noise1(3));

subplot(9,1,k);imshow(img1); k=k+1;
title(['target: [' num2str(round(target1)) ']  noise: [' num2str(round(target1)) ']  dist: ' num2str(round(dist1))]);


[img,target2,noise2,dist2] = img_end(h1,h2,h3);
subplot(9,1,k);imshow(img); k=k+1; 
title(['target: [' num2str(round(target2)) ']  noise: [' num2str(round(target2)) ']  dist: ' num2str(round(dist2))]);

[img,target2,noise2,dist2] = img_end(h1,h2,hc);
subplot(9,1,k);imshow(img); k=k+1; 
title(['target: [' num2str(round(target2)) ']  noise: [' num2str(round(target2)) ']  dist: ' num2str(round(dist2))]);

[img,target2,noise2,dist2] = img_end(h1,hb,h3);
subplot(9,1,k);imshow(img); k=k+1; 
title(['target: [' num2str(round(target2)) ']  noise: [' num2str(round(target2)) ']  dist: ' num2str(round(dist2))]);

[img,target2,noise2,dist2] = img_end(h1,hb,hc);
subplot(9,1,k);imshow(img); k=k+1; 
title(['target: [' num2str(round(target2)) ']  noise: [' num2str(round(target2)) ']  dist: ' num2str(round(dist2))]);

[img,target2,noise2,dist2] = img_end(ha,h2,h3);
subplot(9,1,k);imshow(img); k=k+1; 
title(['target: [' num2str(round(target2)) ']  noise: [' num2str(round(target2)) ']  dist: ' num2str(round(dist2))]);

[img,target2,noise2,dist2] = img_end(ha,h2,hc);
subplot(9,1,k);imshow(img); k=k+1; 
title(['target: [' num2str(round(target2)) ']  noise: [' num2str(round(target2)) ']  dist: ' num2str(round(dist2))]);

[img,target2,noise2,dist2] = img_end(ha,hb,h3);
subplot(9,1,k);imshow(img); k=k+1; 
title(['target: [' num2str(round(target2)) ']  noise: [' num2str(round(target2)) ']  dist: ' num2str(round(dist2))]);

[img,target2,noise2,dist2] = img_end(ha,hb,hc);
subplot(9,1,k);imshow(img); k=k+1; 
title(['target: [' num2str(round(target2)) ']  noise: [' num2str(round(target2)) ']  dist: ' num2str(round(dist2))]);


%% Prob 3 part 2a:  Color-based Image Segmentation
% Step 1. Transfer pixels from RGB to Lab color system
clc
clear
close all


img = imread('Chicken.jpg');
figure()
imshow(img)
title('original')
lab_chicken = rgb2lab(img);
figure()
imshow(lab_chicken)
title('lab color system')

%% Prob 3 part 2b:  Color-based Image Segmentation
% Step 2. Apply k-means clustering on data in a-b domain with cluster number (k=2)

k = 3;
pixel_labels_ab = imsegkmeans(im2single(lab_chicken(:,:,2:3)),k,'NumAttempts',3);

clustered_img = labeloverlay(img,pixel_labels_ab);
imshow(clustered_img)
title('Clustered image with k=3')

%% Prob 3 part 2c:  Color-based Image Segmentation
% Step 3. Erode the segment image to filter out small fragments.


ones_matrix = ones(10);

eroded_label = imerode(pixel_labels_ab,ones_matrix);
clustered_img_eroded = labeloverlay(img,eroded_label);
figure
imshow(clustered_img_eroded)
title('Clustered image eroded')




%% Prob 3 part 3ab:  Principle component analysis (PCA):
%  a. Determine the covariance matrix of data.
% b. Derive the components (eigenvectors) with eigenvalues arranged 
% in a descending order.

clc
clear
close all

img = double(imread('Chicken.jpg'));

X = reshape(img,size(img,1)*size(img,2),3);

[coeff] = pca(X);
eigen_vectors2 = sort(coeff,'descend')

'check conv_matrix & eigen_vectors & eigen_values1' 
R = img(:,:,1);
G = img(:,:,2);
B = img(:,:,3);
 conv_matrix = [conv_cust(R,R) conv_cust(R,G) conv_cust(R,B);
               conv_cust(G,R) conv_cust(G,G) conv_cust(G,B);
               conv_cust(B,R) conv_cust(B,G) conv_cust(B,B)]
[V,D] = eig(conv_matrix);
eigen_vectors1 = sort(V,'descend')
eigen_values1 = sort(diag(D),'descend')

%% Prob 3 part 3c:  Principle component analysis (PCA):
% Obtain the maximum and minimum values of three component matrices. 
% Show these three matrices(images) with linear mapping from the 
% minimum and maximum values to the range of (0-255).

img_new = X*coeff;

matrix = reshape(img_new(:,1),size(img,1),size(img,2));
[uint8_matrix_red,max1,min1] = double_uint8(matrix);
figure, imshow(uint8_matrix_red);
title(['uint8 matrix red ' 'double max: ' num2str(round(max1)) '  double min: ' num2str(round(min1))]);

matrix = reshape(img_new(:,2),size(img,1),size(img,2));
[uint8_matrix_green,max1,min1] = double_uint8(matrix);
figure, imshow(uint8_matrix_green);
title(['uint8 matrix green ' 'double max: ' num2str(round(max1)) '  double min: ' num2str(round(min1))]);


matrix = reshape(img_new(:,3),size(img,1),size(img,2));
[uint8_matrix_blue,max1,min1] = double_uint8(matrix);
figure, imshow(uint8_matrix_blue);
title(['uint8 matrix blue ' 'double max: ' num2str(round(max1)) '  double min: ' num2str(round(min1))]);




%% functions
%




% Problem 2

%TrainingData is the input: 16 inputs, 0 or 1
%  K is input layer:        16 nodes
%    w_pk is K-P weights:   (16x4 matrix)
%  P is second layer:       4 nodes
%    w_qp is P-Q weights:   (4x2 matrix)
%  Q is output layer:       2 nodes
%O_q is the output:         2 outputs, [O;T] confidence from 0 to 1 

%[error_comb,O_p,O_q] = forward(Train_Data(:,k),w_pk ,w_qp,desired);

%function used for forward prob
function [Error,O_p,O_q] = forward(Data,w_pk ,w_qp,desired)

I_k = Data'; %Input k value
O_k = 1.*I_k; %Output k value

I_p = O_k*w_pk; %Input p value
O_p = 1./(1+exp(-I_p)); %output p value

I_q = O_p*w_qp; %input q value
O_q = 1./(1+exp(-I_q))'; %output q value

Error = .5*sum((desired - O_q).^2);

end

%function used to calculate the derivative
function derivative = hprime(O)
derivative = O.*(1-O);
end



% prob 3a


%function used to create a RGB 100x200 image using the R G B as input
% with the target and the noise
function[img,target1,noise1,dist1] = img_start(R1,G1,B1,R2,G2,B2)
img = ones(100,200,3,'uint8');

img(:,1:100,1) = img(:,1:100,1)*R1;
img(:,1:100,2) = img(:,1:100,2)*G1;
img(:,1:100,3) = img(:,1:100,3)*B1;

img(:,101:200,1) = img(:,101:200,1)*R2;
img(:,101:200,2) = img(:,101:200,2)*G2;
img(:,101:200,3) = img(:,101:200,3)*B2;

target1 = [R1,G1,B1];
noise1 = [R2,G2,B2];
dist1 = sqrt(sum((target1 - noise1).^ 2));

end

function[img,target2,noise2,dist2] = img_end(R,G,B)
img = ones(100,200,3,'uint8');


RGB = cat(100,R,G,B);
big_max = max(RGB,[],'all');
big_min = min(RGB,[],'all');

if (big_min<0)
    big_min_matrix = ones(100,200,'double');
    big_min_matrix = big_min_matrix.*big_min;
    %big_max(100,200) = big_max;
    R = R-big_min_matrix;
    G = G-big_min_matrix;
    B = B-big_min_matrix;
end


R(:,1:100) = round(mean(R(:,1:100),'all'));
R(:,101:200) = round(mean(R(:,101:200),'all'));

G(:,1:100) = round(mean(G(:,1:100),'all'));
G(:,101:200) = round(mean(G(:,101:200),'all'));

B(:,1:100) = round(mean(B(:,1:100),'all'));
B(:,101:200) = round(mean(B(:,101:200),'all'));

target2 = [round(mean(R(:,1:100),'all')),round(mean(G(:,1:100),'all')),round(mean(B(:,1:100),'all'))];
noise2 = [round(mean(R(:,101:200),'all')),round(mean(G(:,101:200),'all')),round(mean(B(:,101:200),'all'))];
dist2 = sqrt(sum((target2 - noise2).^ 2));

cast(R,'uint8');
cast(G,'uint8');
cast(B,'uint8');

img(:,:,1) = R;
img(:,:,2) = G;
img(:,:,3) = B;

% subplot(8,2,k);
% imshow(img);
% title(['dist: ' num2str(round(dist2))]);
% k = k+2;
 
end

%prob 3c

% function to calculate the covariance matrix
function[cov_num] = conv_cust(A,B)
[h,w] = size(A);
mean_matrixA = zeros(h,w)+mean(A,'all');
mean_matrixB = zeros(h,w)+mean(B,'all');
matrixA = A - mean_matrixA;
matrixB = B - mean_matrixB;
cov_num = 1/(h*w)*sum(diag(matrixA'*matrixB));
end

% Question 3 part 3c
% function to convert the double matrix to uint8
% using the max and the min values
function [uint8_matrix,max1,min1] = double_uint8(matrix)
max1 = max(matrix,[],'all');
min1 = min(matrix,[],'all');

[h,w] = size(matrix);
min_matrix = ones(h,w,'double');
min_matrix = min_matrix.*min1;
max_matrix = ones(h,w,'double');
max_matrix = max_matrix.*max1;


matrix_new = ((matrix-min_matrix)./(max_matrix-min_matrix))*255;
uint8_matrix = cast(matrix_new,'uint8');
end
