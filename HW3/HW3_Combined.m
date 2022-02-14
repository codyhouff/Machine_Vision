%% HW 3 - ME 6406 Machine Vision
% by Cody Houff
% 11/09/21

%% Problem 1a Camera Model 
% Camera Model 
% find Xw,Yw and u,v
clc
clear
close all

Xw = [-2,-1, 0, 1, 2, 3,-2,-1, 0, 1, 2, 3,-1,-2,-1,-2,-1,-2,-1,-2];
Yw = [ 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5];
Zw = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];



theta = 135*pi/180;


T = [3, 3.5, 7.5]';

f = 1.3;

R = [1,        0,        0;
     0, cos(theta), -sin(theta);
     0   sin(theta),  cos(theta)]

 sizeXw = size(Xw);
 
for n = 1:sizeXw(2)
     
    xyz(:,n) = R*[Xw(n),Yw(n),Zw(n)]'+ T;
    
end

u = f*xyz(1,:)./xyz(3,:); % u = f*x/z
v = f*xyz(2,:)./xyz(3,:); % v = f*y/z

scatter(u,v);
xlabel('u');
ylabel('v');
 
save('camera_calibration_data.mat','Xw','Yw','u','v'); 


%% Problem 1b Camera Calibration
%Find f, [R], T T = offset, R matrix rotation values, given Xw Yw u, v
clc
clear
close all

load('camera_calibration_data.mat');

Zw = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

%A =[Xw', Yw', Zw', u', v']

A = [v'.*Xw', v'.*Yw', -u'.*Xw', -u'.*Yw', v'];

b = u';
mu = pinv(A)*b;
mu(1);
mu(2);

U = mu(1)^2+mu(2)^2+mu(3)^2+mu(4)^2;
Ty = (U -(U^2 - 4*(mu(1)*mu(4) - mu(2)*mu(3))^2)^(.5))/(2*(mu(1)*mu(4)-mu(2)*mu(3))^2);

Ty = sqrt(Ty);
% Ty2 = -sqrt(Ty)

r11 = Ty*mu(1);
r12 = Ty*mu(2);
r21 = Ty*mu(3);
r22 = Ty*mu(4);
Tx=mu(5)*Ty;

Ex = r11*Xw(1)+r12*Yw(1)+Tx;
Ey = r21*Xw(1)+r22*Yw(1)+Ty;


if (sign(Ex) == sign(u(1))) && (sign(Ey) == sign(v(1)))
    Ty = Ty;
else
    Ty = -Ty; 
    r11 = Ty*mu(1);
    r12 = Ty*mu(2);
    r21 = Ty*mu(3);
    r22 = Ty*mu(4);
    Tx=mu(5)*Ty;

    Ex = r11*Xw(1)+r12*Yw(1)+Tx;
    Ey = r21*Xw(1)+r22*Yw(1)+Ty;
end

 s1 = 1;
 s2 = -sign(r11*r12+r12*r22);
 
r13 = s1*sqrt(1-r11^2-r12^2);
r23 = s2*sqrt(1-r21^2-r22^2);

r3 = cross([r11,r12,r13]',[r21,r22,r23]');
r31 = r3(1);
r32 = r3(2);
r33 = r3(3);

R = [r11, r12, r13;
     r21, r22, r23;
     r31, r32, r33]

k1 = 0;

ud = u';
vd = v';

rd_2 = ud.^2+vd.^2;
 
bprime = (r31.*Xw'+r32.*Yw').*ud;

xnew  = r11.*Xw'+r12*Yw'+Tx;

aprime = [xnew rd_2.*xnew -ud];
xprime = pinv(aprime)*bprime;

f = xprime(1)
k = xprime(2)
Tz = xprime(3)
 
%% Problem 2a Eye on hand calibration
% Compute ([Rc12], Tc12) and ([Rc23], Tc23)
clc
clear
close all
 
load('robot_hand_eye_data.mat');

Hc12 = Hc2*inv(Hc1);
Rc12 = Hc12(1:3,1:3)
Tc12 = Hc12(1:3,4)

Hc23 = Hc3*inv(Hc2);
Rc23 = Hc23(1:3,1:3)
Tc23 = Hc23(1:3,4)

    
%% Problem 2b Eye on hand calibration
% (n, θ) for each of the rotation matrixes; 
% [Rc12], [Rc23], [Rg12] and [Rg23].
clc
clear
close all

load('robot_hand_eye_data.mat');

Hc12 = Hc2*inv(Hc1);
Rc12 = Hc12(1:3,1:3);
Tc12 = Hc12(1:3,4);

Hc23 = Hc3*inv(Hc2);
Rc23 = Hc23(1:3,1:3);
Tc23 = Hc23(1:3,4);

Rg12 = Hg12(1:3,1:3);
Tg12 = Hg12(1:3,4);

Rg23 = Hg23(1:3,1:3);
Tg23 = Hg23(1:3,4);

theta_Hc12 = acos((Rc12(1,1)+Rc12(2,2)+Rc12(3,3)-1)/2)
theta_Hc23 = acos((Rc23(1,1)+Rc23(2,2)+Rc23(3,3)-1)/2)
theta_Hg12 = acos((Rg12(1,1)+Rg12(2,2)+Rg12(3,3)-1)/2)
theta_Hg23 = acos((Rg23(1,1)+Rg23(2,2)+Rg23(3,3)-1)/2)

k = [0,0,0]';
for i = 1:3
   theta = theta_Hc12;
   r = Rc12;
   k(i) = sqrt((r(i,i) - cos(theta))/(1-cos(theta))); 
end
n_Rc12 = [sign(r(3,2)-r(2,3)),sign(r(1,3)-r(3,1)),sign(r(2,1)-r(1,2))]'.*k

k = [0,0,0]';
for i = 1:3
   theta = theta_Hc23;
   r = Rc23;
   k(i) = sqrt((r(i,i) - cos(theta))/(1-cos(theta))); 
end
n_Rc23 = [sign(r(3,2)-r(2,3)),sign(r(1,3)-r(3,1)),sign(r(2,1)-r(1,2))]'.*k

k = [0,0,0]';
for i = 1:3
   theta = theta_Hg12;
   r = Rg12;
   k(i) = sqrt((r(i,i) - cos(theta))/(1-cos(theta))); 
end
n_Rg12 = [sign(r(3,2)-r(2,3)),sign(r(1,3)-r(3,1)),sign(r(2,1)-r(1,2))]'.*k

k = [0,0,0]';
for i = 1:3
   theta = theta_Hg23;
   r = Rg23;
   k(i) = sqrt((r(i,i) - cos(theta))/(1-cos(theta))); 
end
n_Rg23 = [sign(r(3,2)-r(2,3)),sign(r(1,3)-r(3,1)),sign(r(2,1)-r(1,2))]'.*k

% Problem 2c Eye on hand calibration
% Find Pc12, Pc23, Pg1, Pg23 
% check [Rc12],[Rc23],[Rg12],[Rg23] from P


Pc12 = 2*sin(theta_Hc12/2)*n_Rc12
Pc23 = 2*sin(theta_Hc23/2)*n_Rc23
Pg12 = 2*sin(theta_Hg12/2)*n_Rg12
Pg23 = 2*sin(theta_Hg23/2)*n_Rg23

Pr = Pc12;
skew_Pr = [0,-Pr(3),Pr(2);Pr(3),0,-Pr(1);-Pr(2),Pr(1),0];
mag_Pr = sqrt(Pr(1)^2+Pr(2)^2+Pr(3)^2);
checked_Rc12 = (1-mag_Pr^2/2)*eye(3)+.5*(Pr*Pr'+sqrt(4-mag_Pr^2)*skew_Pr)

Pr = Pc23;
skew_Pr = [0,-Pr(3),Pr(2);Pr(3),0,-Pr(1);-Pr(2),Pr(1),0];
mag_Pr = sqrt(Pr(1)^2+Pr(2)^2+Pr(3)^2);
checked_Rc23 = (1-mag_Pr^2/2)*eye(3)+.5*(Pr*Pr'+sqrt(4-mag_Pr^2)*skew_Pr)

Pr = Pg12;
skew_Pr = [0,-Pr(3),Pr(2);Pr(3),0,-Pr(1);-Pr(2),Pr(1),0];
mag_Pr = sqrt(Pr(1)^2+Pr(2)^2+Pr(3)^2);
checked_Rg12 = (1-mag_Pr^2/2)*eye(3)+.5*(Pr*Pr'+sqrt(4-mag_Pr^2)*skew_Pr)

Pr = Pg23;
skew_Pr = [0,-Pr(3),Pr(2);Pr(3),0,-Pr(1);-Pr(2),Pr(1),0];
mag_Pr = sqrt(Pr(1)^2+Pr(2)^2+Pr(3)^2);
checked_Rg23 = (1-mag_Pr^2/2)*eye(3)+.5*(Pr*Pr'+sqrt(4-mag_Pr^2)*skew_Pr)


% Problem 2d Eye on hand calibration
% Find Pcg,[Rcg],Tcg

Pr = Pg12+Pc12;
Pc12_minus_Pg12 = Pc12 - Pg12;
skew_Pc12_Pg12 = [0,-Pr(3),Pr(2);Pr(3),0,-Pr(1);-Pr(2),Pr(1),0];

Pr = Pg23+Pc23;
Pc23_minus_Pg23 = Pc23 - Pg23;
skew_Pc23_Pg23 = [0,-Pr(3),Pr(2);Pr(3),0,-Pr(1);-Pr(2),Pr(1),0];

Pcg_prime = pinv([skew_Pc12_Pg12;skew_Pc23_Pg23])*[Pc12 - Pg12;Pc23 - Pg23];

Pcg_prime = round(Pcg_prime)

% "check"
% skew_Pc12_Pg12*Pcg_prime
% skew_Pc23_Pg23*Pcg_prime

mag_Pcg = sqrt(Pcg_prime(1)^2+Pcg_prime(2)^2+Pcg_prime(3)^2);

theta_Pcg = 2*atan(mag_Pcg);

Pcg = Pcg_prime*2*cos(theta_Pcg/2)
% "check"
% Pcg = 2*Pcg_prime/sqrt(1+mag_Pcg^2)

Pr = Pcg;
skew_Pr = [0,-Pr(3),Pr(2);Pr(3),0,-Pr(1);-Pr(2),Pr(1),0];
mag_Pr = sqrt(Pr(1)^2+Pr(2)^2+Pr(3)^2);
Rcg = (1-mag_Pr^2/2)*eye(3)+.5*(Pr*Pr'+sqrt(4-mag_Pr^2)*skew_Pr);
Rcg = round(Rcg)


Tcg = pinv([Rg12-eye(3);Rg23-eye(3)])*[Rcg*Tc12-Tg12;Rcg*Tc23-Tg23]

% % "check"
% [Rg12-eye(3)]*Tcg 
% (Rcg*Tc12-Tg12)
% 
% [Rg23-eye(3)]*Tcg 
% (Rcg*Tc23-Tg23)

% "last check"
% Rcg*Pc12
% Pg12

%% Problem 3 Ellipse-Circle Correspondence
% Find Oc1,Oc2,Oc3,Oc4 
% Nc1,Nc2,Nc3,Nc4 


load('coef2021.mat')
fo = .8690;
r = 6.5;
C = [coef.A, coef.B, coef.D/fo;
     coef.B, coef.C, coef.E/fo;
     coef.D/fo, coef.E/fo, coef.F/fo^2];
[V,D] = eig(C);
P = [V(:,2), V(:,1), V(:,3)];
di = diag(D)*(-1);
a = sqrt(1/di(2));
b = sqrt(1/di(1));
c = sqrt(-1/di(3));

alpha1 = acos((b/a)*sqrt((1+(a/c)^2)/(1+(b/c)^2)))
alpha2 = -alpha1

%gamma1 = ((rc)/(acos(alpha1)))sqrt((1-(b/c)^2tan(alpha1)^2)/(1+tan(alpha1)^4))
%gamma2 = ((rc)/(acos(alpha2)))sqrt((1-(b/c)^2tan(alpha2)^2)/(1+tan(alpha2)^4))

K1 = 1/a^2;
K2 = K1;

gamma_new1 = sqrt((r^2/a^2)/(a^2*((1/b^2)+(1/c^2))^2*(sin(alpha1)^2*cos(alpha1)^2)-((sin(alpha1)^2/b^2)-(cos(alpha1)^2/c^2))));

K3 = (1/(b^2)+1/(c^2))*(sin(2*alpha1))*gamma_new1;
K4 = ((sin(alpha1)^2/b^2)-(cos(alpha1)^2/c^2))*gamma_new1^2;


center1 = P*[1 0 0; 0 cos(alpha1) sin(alpha1); 0 -sin(alpha1) cos(alpha1)]*[0 -K3/(2*K2) gamma_new1]';
normal1 = P*[1 0 0; 0 cos(alpha1) sin(alpha1); 0 -sin(alpha1) cos(alpha1)]*[0 0 1]';

%
gamma_new2 = sqrt((r^2/a^2)/(a^2*((1/b^2)+(1/c^2))^2*(sin(alpha2)^2*cos(alpha2)^2)-((sin(alpha2)^2/b^2)-(cos(alpha2)^2/c^2))));

K3 = (1/(b^2)+1/(c^2))*(sin(2*alpha2))*gamma_new2;
K4 = ((sin(alpha2)^2/b^2)-(cos(alpha2)^2/c^2))*gamma_new2^2;


center2 = P*[1 0 0; 0 cos(alpha2) sin(alpha2); 0 -sin(alpha2) cos(alpha2)]*[0 -K3/(2*K2) gamma_new2]';
normal2 = P*[1 0 0; 0 cos(alpha2) sin(alpha2); 0 -sin(alpha2) cos(alpha2)]*[0 0 1]';

%
alpha3 = alpha1 +pi

gamma_new3 = sqrt((r^2/a^2)/(a^2*((1/b^2)+(1/c^2))^2*(sin(alpha3)^2*cos(alpha3)^2)-((sin(alpha3)^2/b^2)-(cos(alpha3)^2/c^2))));

K3 = (1/(b^2)+1/(c^2))*(sin(2*alpha3))*gamma_new3;
K4 = ((sin(alpha3)^2/b^2)-(cos(alpha3)^2/c^2))*gamma_new3^2;


center3 = P*[1 0 0; 0 cos(alpha3) sin(alpha3); 0 -sin(alpha3) cos(alpha3)]*[0 -K3/(2*K2) gamma_new3]';
normal3 = P*[1 0 0; 0 cos(alpha3) sin(alpha3); 0 -sin(alpha3) cos(alpha3)]*[0 0 1]';

%

alpha4 = alpha2 +pi

gamma_new4 = sqrt((r^2/a^2)/(a^2*((1/b^2)+(1/c^2))^2*(sin(alpha4)^2*cos(alpha4)^2)-((sin(alpha4)^2/b^2)-(cos(alpha4)^2/c^2))));

K3 = (1/(b^2)+1/(c^2))*(sin(2*alpha4))*gamma_new4;
K4 = ((sin(alpha4)^2/b^2)-(cos(alpha4)^2/c^2))*gamma_new4^2;

center4 = P*[1 0 0; 0 cos(alpha4) sin(alpha4); 0 -sin(alpha4) cos(alpha4)]*[0 -K3/(2*K2) gamma_new4]';
normal4 = P*[1 0 0; 0 cos(alpha4) sin(alpha4); 0 -sin(alpha4) cos(alpha4)]*[0 0 1]';


Oc1_Oc2_Oc3_Oc4 = [center1,center2,center3,center4]

Nc1_Nc2_Nc3_Nc4 = [normal1,normal2,normal3,normal4]


%% Problem 4a Morphology
% 
clc
clear
clear tabs

img = imread('Fingerprint.jpg');
figure()
imshow(img);

SE = [1,1,1;
    1,1,1;
    1,1,1];

img_erode = imerode(img,SE);
figure()
imshow(img_erode);

%% Problem 4b Morphology
% 

img_dilate_erode = imdilate(img_erode,SE);
figure()
imshow(img_dilate_erode);

%% Problem 4c Morphology
% 

img_dilate_dilate_erode = imdilate(img_dilate_erode,SE);
figure()
imshow(img_dilate_dilate_erode);

%% Problem 4d Morphology
% 

img_erode_dilate_dilate_erode = imerode(img_dilate_dilate_erode,SE);
figure()
imshow(img_erode_dilate_dilate_erode);

%% Problem 2d practice (extra)
% % 
% clc
% clear
% clear tabs
% 
% 
% %axb = c
% a1 = [1,2,3]'
% a2 = [3,4,2]'
% %b = [4,5,6]'
% c1 = [-3,6,-3]'
% c2 = [14,-10,-1]'
% % c = cross(a,b)
% 
% skew_a1= [0,-a1(3),a1(2);a1(3),0,-a1(1);-a1(2),a1(1),0]
% 
% skew_a2= [0,-a2(3),a2(2);a2(3),0,-a2(1);-a2(2),a2(1),0]
% 
% skew_a12 = [skew_a1;skew_a2]
% c12 = [c1;c2]
% b_new = pinv(skew_a12)*c12





