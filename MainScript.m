
%Begin MainScript.m
%MainScript.m written by Dr. Hung La
%Additions to MainScript.m noted by Brendan Aguiar
%Initialization
clc
close all
clear

%Load data and correct IMU initial offset
[time, data] = rtpload('EKF_DATA_circle.txt');%data of the circle in front of engineer building
Odom_x = data.O_x;
Odom_y = data.O_y;
Odom_theta = data.O_t;
Gps_x = data.G_x;
Gps_y = data.G_y;
Gps_Co_x = data.Co_gps_x;%Variable added by Brendan Aguiar
Gps_Co_y = data.Co_gps_y;%Variable added by Brendan Aguiar
IMU_heading = data.I_t;
IMU_Co_heading = data.Co_I_t;

%noise variable assignment written by Brendan Aguiar
noise_mean = 5;
noise_std = 2.5;
noise = noise_std .* randn(length(Odom_x), 2) + noise_mean .* ones(length(Odom_x), 2);
n = input('Choose plotting option: \n1. Generate Normal Kalman Filter\n2. Generate w/ Gps Cov noise\n3. Generate w/ IMU Cov noise\n4. Generate w/ GPS Pos noise\n5. GPS Cov/Pos noise\n');
switch n
    case 1
    case 2
        Gps_Co_xN = data.Co_gps_x + noise(:,1);
        Gps_Co_yN = data.Co_gps_y + noise(:,2);
    case 3
        IMU_Co_headingN = IMU_Co_heading + noise(:, 1);
    case 4
        Gps_xN = data.G_x + noise(:,1);
        Gps_yN = data.G_y + noise(:,2);
    case 5
        Gps_xN = data.G_x + noise(:,1);
        Gps_yN = data.G_y + noise(:,2);
        Gps_Co_xN = data.Co_gps_x + noise(:,1);
        Gps_Co_yN = data.Co_gps_y + noise(:,2);
end
      
%Calibrate IMU to match with robot's heading initially
IMU_heading = IMU_heading + (.32981-.237156)*ones(length(IMU_heading),1);%For EKF_DATA3

%Velocity of the robot
V = .14;
%Distance between 2 wheels
L = 1;%meter
%Angular Velocity
Omega = V*tan(Odom_theta(1))/L;%Angular Velocity
%set time_step
delta_t = .001;
%total_step
total=1:length(Odom_x);
s.x = [Odom_x(1); Odom_y(1); V; Odom_theta(1); Omega]; %Enter State (1x5)

%Enter transition matrix A (5x5)
s.A = [1 0 delta_t*cos(Odom_theta(1)) 0 0;  
    0 1 delta_t*sin(Odom_theta(1)) 0 0;
    0 0 1                       0 0;
    0 0 0                       1 delta_t;
    0 0 0                       0 1];

%Enter covariance matrix Q (5x5) for state x
s.Q = [.00004 0 0 0 0;
    0 .00004 0 0 0;
    0 0 .0001 0 0;
    0 0 0 .0001 0;
    0 0 0 0 .0001];

%Enter measurement matrix H (5x5) for measurement model z
s.H = eye(5); %Implemented eye function for identity matrices - Brendan Aguiar

%Enter covariance matrix R (5x5) for measurement model Z
s.R = [.04 0 0 0 0;
    0 .04 0 0 0;
    0 0 .01 0 0;
    0 0 0 .01 0;
    0 0 0 0 .01];

%B matrix initialization:
s.B = eye(5); 

%Enter initial value of u (5x5)
s.u = [0;0;0;0;0];

%Enter initial covariance matrix P (5x5)
s.P = .001 * eye(5);%Scalar multiplied by identity matrix - Brendan Aguiar
%sPlus1 = KalmanFilter(s);

%*Data for Plots*%
true = [];      %truth voltage
X1 = [];
X2 = [];
X_heading = [];
%*Start Kalman Filter*%
%For the following switch statement, place a capital N at the end of data you
%are adding noise to (ie. Gps_xN). If no designation for noise is given the
%switch case will process the data without noise. - Brendan Aguiar
m = input('Choose how to distribute noise:\n1. Distribute noise to period of set\n2. Distribute noise to entire set\n');
switch m
    case 1
        j = length(total) / 4;%partition start pointof data set
        k = length(total) * 3 / 4;%partition end point
        for t=1:j
            %Enter transition matrix A(5x5)
            s(t).A = [1 0 delta_t*cos(Odom_theta(t)) 0 0;
                0 1 delta_t*sin(Odom_theta(t))      0 0;
                0 0 1                               0 0;
                0 0 0                       1 delta_t;
                0 0 0                       0 1];
            %Enter transition matrix A(5x5)
            s(t).A = [1 0 delta_t*cos(IMU_heading(t)) 0 0;
                0 1 delta_t*sin(IMU_heading(t)) 0 0; 
                0 0 1                           0 0;
                0 0 0                   1 delta_t;
                0 0 0                   0 1];
            %Enter covariance matrix R(5x5) for measurement model z
            s(t).R = [Gps_Co_x(t)   0   0   0 0;
                0   Gps_Co_y(t)     0       0 0;
                0   0   .01                 0 0;
                0   0   0   IMU_Co_heading(t) 0;
                0   0   0   0               .01];
            s(t).z = [Gps_x(t); Gps_y(t); V; IMU_heading(t); Omega];
         %State Prediction
        s(t+1) = KalmanFilter(s(t));%Kalman filter to predict future pos. - Brendan Aguiar

        X = s(t).x;
        X1(t,:) = X(1,:);
        X2(t,:) = X(2,:);

        X_theta = s(t).x;
        X_heading(t,:) = X_theta(4,:);
        end
        for t=j:k %introduce noise
            %Enter transition matrix A(5x5)
            s(t).A = [1 0 delta_t*cos(Odom_theta(t)) 0 0;
                0 1 delta_t*sin(Odom_theta(t))      0 0;
                0 0 1                               0 0;
                0 0 0                       1 delta_t;
                0 0 0                       0 1];
            %Enter transition matrix A(5x5)
            s(t).A = [1 0 delta_t*cos(IMU_heading(t)) 0 0;
                0 1 delta_t*sin(IMU_heading(t)) 0 0; 
                0 0 1                           0 0;
                0 0 0                   1 delta_t;
                0 0 0                   0 1];
            %Enter covariance matrix R(5x5) for measurement model z
            s(t).R = [Gps_Co_x(t)   0   0   0 0;
                0   Gps_Co_y(t)     0       0 0;
                0   0   .01                 0 0;
                0   0   0   IMU_Co_heading(t) 0;
                0   0   0   0               .01];
            s(t).z = [Gps_x(t); Gps_y(t); V; IMU_heading(t); Omega];
         %State Prediction
        s(t+1) = KalmanFilter(s(t));%Kalman filter to predict future pos. - Brendan Aguiar

        X = s(t).x;
        X1(t,:) = X(1,:);
        X2(t,:) = X(2,:);

        X_theta = s(t).x;
        X_heading(t,:) = X_theta(4,:);
        end
        for t=k:length(total)
            %Enter transition matrix A(5x5)
            s(t).A = [1 0 delta_t*cos(Odom_theta(t)) 0 0;
                0 1 delta_t*sin(Odom_theta(t))      0 0;
                0 0 1                               0 0;
                0 0 0                       1 delta_t;
                0 0 0                       0 1];
            %Enter transition matrix A(5x5)
            s(t).A = [1 0 delta_t*cos(IMU_heading(t)) 0 0;
                0 1 delta_t*sin(IMU_heading(t)) 0 0; 
                0 0 1                           0 0;
                0 0 0                   1 delta_t;
                0 0 0                   0 1];
            %Enter covariance matrix R(5x5) for measurement model z
            s(t).R = [Gps_Co_x(t)   0   0   0 0;
                0   Gps_Co_y(t)     0       0 0;
                0   0   .01                 0 0;
                0   0   0   IMU_Co_heading(t) 0;
                0   0   0   0               .01];
            s(t).z = [Gps_x(t); Gps_y(t); V; IMU_heading(t); Omega];
         %State Prediction
        s(t+1) = KalmanFilter(s(t));%Kalman filter to predict future pos. - Brendan Aguiar

        X = s(t).x;
        X1(t,:) = X(1,:);
        X2(t,:) = X(2,:);

        X_theta = s(t).x;
        X_heading(t,:) = X_theta(4,:);
        end
        case 2
        for t=1:length(total)
            %Enter transition matrix A(5x5)
            s(t).A = [1 0 delta_t*cos(Odom_theta(t)) 0 0;
                0 1 delta_t*sin(Odom_theta(t))      0 0;
                0 0 1                               0 0;
                0 0 0                       1 delta_t;
                0 0 0                       0 1];
            %Enter transition matrix A(5x5)
            s(t).A = [1 0 delta_t*cos(IMU_heading(t)) 0 0;
                0 1 delta_t*sin(IMU_heading(t)) 0 0; 
                0 0 1                           0 0;
                0 0 0                   1 delta_t;
                0 0 0                   0 1];
            %Enter covariance matrix R(5x5) for measurement model z
            s(t).R = [Gps_Co_x(t)   0   0   0 0;
                0   Gps_Co_y(t)     0       0 0;
                0   0   .01                 0 0;
                0   0   0   IMU_Co_heading(t) 0;
                0   0   0   0               .01];
            s(t).z = [Gps_x(t); Gps_y(t); V; IMU_heading(t); Omega];
         %State Prediction
        s(t+1) = KalmanFilter(s(t));%Kalman filter to predict future pos. - Brendan Aguiar

        X = s(t).x;
        X1(t,:) = X(1,:);
        X2(t,:) = X(2,:);

        X_theta = s(t).x;
        X_heading(t,:) = X_theta(4,:);
        end
end

%*Plot the Position*%
figure(1)
hold on
grid on
%plot Odometry(x,y) data:
hz = plot(Odom_x, Odom_y, '.r');
%plot Gps (x,y) data:
hgps = plot(Gps_x, Gps_y, '.k');
%plot KF estimates:
hk=plot(X1, X2,'.b');
legend([hz hgps hk],'Odometry','gps calibrated','Kalman output', 'Location', 'Best')
title('Fusion of GPS+IMU and ODOMETRY in Position')

figure(2)
hold on
grid on
odom_heading=plot(time, data.O_t, 'r');
imu_heading=plot(time, IMU_heading, 'k');
KF_heading = plot(X_heading, 'b');
legend([odom_heading, imu_heading, KF_heading],'Odometry heading','IMU heading', 'KF heading','Location', 'Best')
title('Fusion of GPS+IMU and ODOMETRY in heading')
%End MainScript.m

