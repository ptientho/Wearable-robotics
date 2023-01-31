% ME655-WS(BME656-WS) - Virtual Lab Experience 3
% last rev. Oct 2022

clear all
close all
clc

% Create Simulation data
load('ME655_Lect09_VL3_Data.mat')
LH_hip_traj = timeseries([Lhip_pos Lhip_vel Lhip_acc Rhip_pos Rhip_vel Rhip_acc],t);

% initial conditions for human controller
Lhip_pos0=Lhip_pos(1);
Lhip_vel0=Lhip_vel(1);
Tau_h_max=30; %[Nm] max torque human can exert

%% Robot motor control

% Oscillator and filter settings:
eps=12;
nu=.5;
M=6;
lambda=.95;
N=80;
h=2.5*N;

t_start=5.0; %[s] time of AFO activation
dt_active_control=10; %[s] wait 10s after AFO is active before turning assistance ON
%% CLME Training

%select train dataset
train_indices = find(t<=30);
Lhip_pos_train = Lhip_pos(train_indices(:));
Lhip_vel_train = Lhip_vel(train_indices(:));
Rhip_pos_train = Rhip_pos(train_indices(:));
Rhip_vel_train = Rhip_vel(train_indices(:));

%normalize data
[xp,xh,LhipAvg,RhipAvg,Sp,Sh] = normalize_data(Lhip_pos_train,Lhip_vel_train,Rhip_pos_train,Rhip_vel_train);

%calculate Mhp and Mhh
Mhp = nan(size(xh,2),size(xp,2));
Mhh = nan(size(xh,2),size(xh,2));
n = length(xp(:,1)); 
for i = 1:length(Mhp(:,1))
    for j = 1:length(Mhp(1,:))
        Mhp(i,j) = sum(xh(:,i).*xp(:,j))/(n-1);
    end
end

for i = 1:length(Mhh(:,1))
    for j = 1:length(Mhh(1,:))
        Mhh(i,j) = sum(xh(:,i).*xh(:,j))/(n-1);
    end
end

%calculate C, K, and k
C = (Mhh \ Mhp)';
K = Sp*C*inv(Sh);
k = -K*RhipAvg + LhipAvg;

%given K and k, estimate from BLUE
Lhip_pred = zeros(length(train_indices),2);
Rhip_train = [Rhip_pos_train, Rhip_vel_train];
for i = 1:length(Lhip_pred(:,1))
    Lhip_pred(i,:) = K*Rhip_train(i,:)'+k;
end
%% Kalman Filter

T = t(2)-t(1); %sample period
A = [1 T;0 1]; G = [T^2/2; T]; %process parameters
Q = G*G'*var(Lhip_acc(train_indices(:))); %process noise covariances
H = eye(2); %measure state directly
R = [var(Lhip_pred(:,1)-Lhip_pos_train) 0;0 var(Lhip_pred(:,2)-Lhip_vel_train)]; %measurement noise covariance
