%% Clear past plots, variables and commands
close all; clear all; clc;

% Load data 
load 'EE5114_CA1.mat';

% acx = x-axis accelerometer reading
% acy = y-axis accelerometer reading
% acz = z-axis accelerometer reading
% 
% phi = Roll angle computed by the drone's on-board computer
% tht = Pitch angle computed by the drone's on-board computer
% psi = Yaw angle computed by the drone's on-board computer 
% 
% fix = GPS position fix signal 
% eph = GPS horizontal variance 
% epv = GPS vertical variance 
% lat = GPS Latitude
% lon = GPS Longitude
% alt = GPS altitude
% gps_nSat = Number of GPS satellites
% 
% out1 = Motor 1 signal
% out2 = Motor 2 signal
% out3 = Motor 3 signal
% out4 = Motor 4 signal

%% Accelerometer plot
figure; set(gcf,'numbertitle','off','name','Acceleration');  
subplot(3,1,1); plot(t, acx, 'b'); ylim([-2 2]); ylabel('acx (m/s^2)'); grid on; 
subplot(3,1,2); plot(t, acy, 'b'); ylim([-2 2]); ylabel('acy (m/s^2)'); grid on; 
subplot(3,1,3); plot(t, acz, 'b'); ylabel('acz (m/s^2)'); grid on; 

%% Euler angles plot
figure; set(gcf,'numbertitle','off','name','Euler Angles');  
subplot(3,1,1); plot(t, rad2deg(phi), 'b'); ylabel('Roll (degree)'); grid on; 
subplot(3,1,2); plot(t, rad2deg(tht), 'b'); ylabel('Pitch (degree)'); grid on; 
subplot(3,1,3); plot(t, rad2deg(psi), 'b'); ylabel('Yaw (degree)'); grid on; 

%% GPS plot
figure; set(gcf,'numbertitle','off','name','GPS');  
subplot(3,2,1); plot(t, lon); ylabel('Longitude'); grid on;
subplot(3,2,3); plot(t, lat); ylabel('Latitude'); grid on;
subplot(3,2,5); plot(t, alt); ylabel('Altitude'); grid on; xlabel('time (s)');

subplot(3,2,2); plot(t, gps_nSat, '.'); ylabel('Sat'); grid on;
subplot(3,2,4); plot(t, eph); ylabel('Eph'); grid on; ylim([0 5]);
subplot(3,2,6); plot(t, epv); ylabel('Epv'); grid on; ylim([0 5]);

%% Motor signal plot
figure; set(gcf,'numbertitle','off','name','Motor Signal');  
hold on;
plot(t,out1,'r');
plot(t,out2,'g');
plot(t,out3,'b');
plot(t,out4,'y');
legend('Motor1','Motor2','Motor3','Motor4'); 
ylabel('Motor inputs'); xlabel('time (s)'); ylim([1000 2000]); grid on;


%%%%%%%%%%%%%%%%%%%%%% Your own coding work start from here %%%%%%%%%%%%%%%%%%%%%%%%%

%% Convert GPS raw measurements to local NED position values

%Matric number: A0285139W

t_seg = t(4129:9753);
a = 6378137;
b = 6356752;
e = sqrt(a^2-b^2)/a;
Ne = a./sqrt(1-(e^2).*((sind(lat)).^2));
%The eccentricity and Ne calculation

Pe = zeros(3,10215);
for i = 1:10215
    Xe = ((Ne(i)+alt(i)).*cosd(lat(i)).*cosd(lon(i)));
    Ye = ((Ne(i)+alt(i)).*cosd(lat(i)).*sind(lon(i)));
    Ze = ((Ne(i)*(1-e^2)+alt(i)).*sind(lat(i)));
    Pe(1,i) = Xe;
    Pe(2,i) = Ye;
    Pe(3,i) = Ze;
end % convert the GPS data to ECEF frame.

Pn = zeros(3,10215);
for i = 1:10215
    Rn_e = [-sind(lat(1))*cosd(lon(1)), -sind(lat(1))*sind(lon(1)), cosd(lat(1));
            -sind(lon(1)),             cosd(lon(1)),             0;
            -cosd(lat(1))*cosd(lon(1)), -cosd(lat(1))*sind(lon(1)), -sind(lat(1))]; % define the rotational matrix, the ref value is the first set of ECEF data.

    Pn(:,i) = Rn_e*(Pe(:,i)-Pe(:,1)); % convert the data in ECEF frame to NED frame.
end


figure; set(gcf,'numbertitle','off','name','measured NED value');  
subplot(3,1,1); plot(t, Pn(1,:)); ylabel('NED X'); grid on;
subplot(3,1,2); plot(t, Pn(2,:)); ylabel('NED Y'); grid on;
subplot(3,1,3); plot(t, Pn(3,:)); ylabel('NED Z'); grid on; xlabel('time (s)');


%% Implement EKF to estimate NED position and velocity

X = [3.0401e+05, -3.6153e+04, 6.0413e+03,0,0,0,0,0,0]';% the initial estimated states
P = eye(9);
time = 5624; % The total time between 10 minutes before the 2nd took off and 5 min after landing.

Gravity = [0;
           0;
           9.81];% define the gravity component in G matrix

H = eye(3,9);
% measurement function

Q = abs([2.5*randn(3,1).*eye(3,3),zeros(3,1);
    zeros(1,4)]);
R = abs(0.1*randn(3,1).*eye(3,3));
% Define the gaussian noise and the standard deviation of it

estimated = zeros(9,9753);

for j = 4129:9753
    dt = abs(t(j)-t(j-1));

    F1 = [diag([1,1,1]),diag([dt,dt,dt]),diag([(-dt^2)/2,(-dt^2)/2,(-dt^2)/2]);
         zeros(3,3),diag([1,1,1]),diag([-dt,-dt,-dt])];
    % Define the part of controllability matrix

    G1 = [diag([(dt^2)/2,(dt^2)/2,(dt^2)/2]);
         diag([dt,dt,dt]);
         zeros(3,3)];
    % Define the part observability matrix

    RotationMatrix = [cos(psi(j))*cos(tht(j)), cos(psi(j))*sin(tht(j))*sin(phi(j))-sin(psi(j))*cos(phi(j)), cos(psi(j))*sin(tht(j))*cos(phi(j))+sin(psi(j))*sin(phi(j));
                      sin(psi(j))*cos(tht(j)), sin(psi(j))*sin(tht(j))*sin(phi(j))+cos(psi(j))*cos(phi(j)), sin(psi(j))*sin(tht(j))*cos(phi(j))-cos(psi(j))*sin(phi(j));
                      -sin(tht(j)),            cos(tht(j))*sin(phi(j)),                                     cos(tht(j))*cos(phi(j))                                        
                      ];

    Fpart = [eye(6,6), zeros(6,3); zeros(3,6), RotationMatrix];
    %Define the second part of F matrix

    Gpart = [RotationMatrix,Gravity];
    %Define the second part of G matrix

    input_U = [acx(j);
                    acy(j);
                    acz(j);
                    1];

    F2 = F1*Fpart;
    F = [F2;
        zeros(3,3), zeros(3,3), eye(3,3)];
    % construct the controllability matrix
    G = G1*Gpart;
    % construct the observability matrix

    % Predicting the new estimated value and the P matrix
    X_predicted = F * X + G*input_U;
    p_predicted = F * P * F'+ G*Q*G';

    %calculating the kalman gain
    K = p_predicted *H'*((H*p_predicted*H'+R)^(-1));

    %Updating the X and P values
    X = X_predicted + K * (Pn(:,j)-H*X_predicted);
    P = p_predicted - K * H * p_predicted;

    %Store the X matrix in the estimated values
    estimated(:,j) = X;
end




%% Result plots
figure; set(gcf,'numbertitle','off','name','estimated NED');
subplot(3,2,1); plot(t_seg, estimated(1,4129:9753).', 'b'); ylabel('NED X'); grid on; 
subplot(3,2,3); plot(t_seg, estimated(2,4129:9753).', 'b'); ylabel('NED Y'); grid on; 
subplot(3,2,5); plot(t_seg, estimated(3,4129:9753).', 'b'); ylabel('NED Z'); grid on; xlabel('time/s')
%plot the estimated NED positions

subplot(3,2,2); plot(t_seg, estimated(4,4129:9753).', 'r'); ylabel('NED X vel'); grid on; 
subplot(3,2,4); plot(t_seg, estimated(5,4129:9753).', 'r'); ylabel('NED Y vel'); grid on; 
subplot(3,2,6); plot(t_seg, estimated(6,4129:9753).', 'r'); ylabel('NED Z vel'); grid on; xlabel('time/s')
%plot the estimated NED velocity

figure; set(gcf,'numbertitle','off','name','bias');
subplot(3,1,1); plot(t_seg, estimated(7,4129:9753)); ylabel('bias X'); grid on;
subplot(3,1,2); plot(t_seg, estimated(8,4129:9753)); ylabel('bias Y'); grid on;
subplot(3,1,3); plot(t_seg, estimated(9,4129:9753)); ylabel('bias Z'); grid on; xlabel('time (s)');
%plot the acceleration bias 






