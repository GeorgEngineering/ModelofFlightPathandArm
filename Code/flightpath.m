function [VEx,VEy,aEx,aEy,VCx,aCx,om_CD,alp_CD] = lab2;
%
% 1. There are 5000 rows and 7 columns in NFCdata.txt. The 7 columns are 
%    etx, ety, enx, eny, rho, t, s, respectively, where, for 0 <= x <= 20
% 2. etx,ety: (x,y) components of the tangential basis vector et, 
% 3. enx,eny: (x,y) components of the normal basis vector en, 
% 4. rho: radius of curvature ((1+y'^2)^1.5/|y''|) for, 
% 5. t: the time instants of NFC at location x, and
% 6. s: the arc length of travel along the flight path y = sqrt(100-x)

% Load NFCdata.txt and define coor and travel info of NFC

x = linspace(0,20,5000)';
y_NFC = sqrt(100-x);
nx = length(x);

load NFCdata.txt;

etx=NFCdata(:,1);
ety=NFCdata(:,2);
enx=NFCdata(:,3);
eny=NFCdata(:,4);
rho=NFCdata(:,5);
t=NFCdata(:,6);
s=NFCdata(:,7);

theta=150/180*pi;
V0_NFC = 4;
at_NFC = V0_NFC/t(end);

% NFC velocity, accel at location x of the capture zone

V_NFC = sqrt(16 - 2*at_NFC*s);   % TODO 1
VEx = V_NFC.*etx;
VEy = V_NFC.*ety;
an_NFC = V_NFC.^2./rho;          % TODO 2
aEx = at_NFC.*etx + an_NFC.*enx;
aEy = at_NFC.*ety + an_NFC.*eny;

% Calculate base arm kinematics: phi, om_CD
phi = asin((y_NFC-(10*sind(30)))/10);  % TODO 3 - phi
om_CD = VEy./(10*cos(phi));            % TODO 4 - phi dot 
alp_CD = (aEy+10*(om_CD.*om_CD)).*sin(phi)./(10*cos(phi));  % TODO 5 - phi double dot

% Calculate point C kinematics of the base arm movement: VCx, aCx

VCx = VEx+(10*om_CD.*sin(phi));     % TODO 6
aCx = aEx+(10.*alp_CD.*sin(phi))+(10*om_CD.*om_CD.*cos(phi));    % TODO 7

% (FIGURE 1): Plotting V_Ex vs. x
plot(x, VEx);
title('1. V_{Ex} vs. x');
xlabel('x');
ylabel('V_{Ex}');

% (FIGURE 2): Plotting V_Ey vs. x
plot(x, VEy);
title('2. V_{Ey} vs. x');
xlabel('x');
ylabel('V_{Ey}');

% (FIGURE 3): Plotting a_Ex vs. x
plot(x, aEx);
title('3. a_{Ex} vs. x');
xlabel('x');
ylabel('a_{Ex}');

% (FIGURE 4): Plotting a_Ey vs. x
plot(x, aEy);
title('4. a_{Ey} vs. x');
xlabel('x');
ylabel('a_{Ey}');

% (FIGURE 5): Plotting V_Cx vs. x
% plot(x, VCx);
% title('5. V_{Cx} vs. x');
% xlabel('x');
% ylabel('V_{Cx}');

% (FIGURE 6): Plotting a_Cx vs. x
plot(x, aCx);
title('6. a_{Cx} vs. x');
xlabel('x');
ylabel('a_{Cx}');

% (FIGURE 7): Plotting om_CD or theta' vs. x
plot(x, om_CD);
title('7. \omega_{CD} (theta.) vs. x');
xlabel('x');
ylabel('\omega_{CD}');

% (FIGURE 8): Plotting alp_CD (theta'') vs. x
plot(x, alp_CD);
title('8. \alpha_{CD} (theta..) vs. x');
xlabel('x');
ylabel('\alpha_{CD}');