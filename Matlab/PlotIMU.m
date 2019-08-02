%===================================================
% Program created By Jesus Adrian Gutierrez 
%   
% Date July 18th, 2019
%   
% " An Open-Source Platform for an Augmented Reality Interface 
%   for Users of an Upper Extremity Rehabilitation Robot "
%   
% Rehabilitaion Institude, Loma Linda University
%
% The purpose of this program is to plot in 3D the 
% motion through space of a multi-linked object. Primaryly to be
% to be used along with the data gathered by "trueIMU" arduino
% motion tracking wearable unit.
%====================================================

clear;
clc ;
close all;

trajectory = load('prelimTest3');    % loads data into a struct with a table of strings inside
trajecto   = trajectory.DATA;        % Needed to access Data inside struct  
traject    = table2array(trajecto);  % Convert data type String to Double


motionA  = traject(:,1:7);           % All data from IMU A Rotation in Quanterion & Lin accelerations (m/s^2)
rotquatA = motionA(:,1:4);
linaccA  = motionA(:,5:7);
motionB  = traject(:,8:14);          % All data from IMU B Rotation in Quanterion & Lin accelerations (m/s^2)
rotquatB = motionB(:,1:4);
linaccB  = motionB(:,5:7);

timestep   = 0.200;                  % Time step defined by Arduino code IMU taking data every 200 millisecs
m = length(motionA);                 % Amount of points that will be plotted 


velA  = zeros(size(linaccA));         % Initialize velocity, step size, and position arrays
poseA = zeros(size(linaccA));
velB  = zeros(size(linaccB));            
poseB = zeros(size(linaccB));
trueA = zeros(size(linaccA)); 
trueB = zeros(size(linaccB)); 
R     = zeros(3,3);           
RAB   = zeros(3,3,m); 

%====== Trasnform linear accelerations to intertial reference frame ======%
for i = 1 : 1: m
    q = rotquatA(i,:);
   
    R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
    R(1,2,:) = 2.*(q(:,2).*q(:,3)+q(:,1).*q(:,4));
    R(1,3,:) = 2.*(q(:,2).*q(:,4)-q(:,1).*q(:,3));
    R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
    R(2,2,:) = 2.*q(:,1).^2-1+2.*q(:,3).^2;
    R(2,3,:) = 2.*(q(:,3).*q(:,4)+q(:,1).*q(:,2));
    R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
    R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
    R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;
    
    trueA(i,:) = R * linaccA(i,:)'  ;
    RA(:,:,i) = R;
    RAB(:,:,i) = R;

end
for i = 1 : 1: m
    q = rotquatB(i,:);
   
    R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
    R(1,2,:) = 2.*(q(:,2).*q(:,3)+q(:,1).*q(:,4));
    R(1,3,:) = 2.*(q(:,2).*q(:,4)-q(:,1).*q(:,3));
    R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
    R(2,2,:) = 2.*q(:,1).^2-1+2.*q(:,3).^2;
    R(2,3,:) = 2.*(q(:,3).*q(:,4)+q(:,1).*q(:,2));
    R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
    R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
    R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;
    
    RB(:,:,i) = RAB(:,:,i) * R ;
    trueB(i,:) = RB(:,:,i) * linaccA(i,:)' ;
end

%====== Integrate once to obtain Velocity of IMUs ======%
for i = 2 : 1: m
    velA(i,:) = velA(i-1,:) + (trueA(i,:) * timestep) ;
    velB(i,:) = velB(i-1,:) + (trueB(i,:) * timestep) ;
end
%====== Integrate once again to get Position of IMUs ======%
for i = 2 : 1 : m
    poseA(i,:) = poseA(i-1,:) + ( velA(i,:) * timestep ) + (0.5 * trueA(i,:) * timestep * timestep);
    poseB(i,:) = poseB(i-1,:) + ( velB(i,:) * timestep ) + (0.5 * trueB(i,:) * timestep * timestep);
end


%====== Plot Setting ======%
length = 0.06;
aviobj = VideoWriter('test.avi');
open(aviobj);
scrsz = get(0,'ScreenSize');
%ScreenSize is a four-element vector: [left, bottom, width, height]:
fig=figure('Position',[1 scrsz(4)/2 scrsz(3)/2 scrsz(4)/2]);
t8itle_handle = title('IMU tracker plot');

itv=20;
rotation_spd=0.4;
delay=0.08;

az=15;
el=64;
view(az,el);
grid on;
xlabel('x', 'fontsize',16);
ylabel('y', 'fontsize',16);
zlabel('z', 'fontsize',16);

%====== 3D Plotting of IMU ======%
for i = 1 : 1 : 39
    
    % Generate axis vectors
    tx = [length,0.0,0.0];
    ty = [0.0,length,0.0];
    tz = [0.0,0.0,length];
    
    % Rotate it by Rotation of IMU A 
    t_x_new = RA(:,:,i)*tx';
    t_y_new = RA(:,:,i)*ty';
    t_z_new = RA(:,:,i)*tz';
    
      % translate vectors to camera position. Make the vectors for plotting
    origin=[poseA(i,1), poseA(i,2), poseA(i,3)];
    tx_vec(1,1:3) = origin;
    tx_vec(2,:) = t_x_new + origin';
    ty_vec(1,1:3) = origin;
    ty_vec(2,:) = t_y_new + origin';
    tz_vec(1,1:3) = origin;
    tz_vec(2,:) = t_z_new + origin';
    
    hold on;
    % Plot the direction vectors at the point
    p1=plot3(tx_vec(:,1), tx_vec(:,2), tx_vec(:,3));
    set(p1,'Color','Green','LineWidth',1);
    p1=plot3(ty_vec(:,1), ty_vec(:,2), ty_vec(:,3));
    set(p1,'Color','Blue','LineWidth',1);
    p1=plot3(tz_vec(:,1), tz_vec(:,2), tz_vec(:,3));
    set(p1,'Color','Red','LineWidth',1);
    
    az=az+rotation_spd;
    view(az,el);
    pause(delay);  % in second
    
end

close(aviobj);
fprintf('Done\n');
