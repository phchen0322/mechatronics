clc
clear
close all


%% INPUTS
%% Equations of motion
% Constants
mass = 18; %Mass
length = 0.97; %Length
beam = 0.32; %Width
x_u_dot=1.2;
y_v_dot=66.1;
N_r_dot=2.3;
M_v_dot=0; Y_r_dot=0;
d_33_c1 = 122.3;
d_33_c2 = 4.8;


% Estimation of d11 d22 d33
Xdata_d11=importdata('Xforce.txt'); %Damping in surge
X_coef_d11=polyfit(Xdata_d11(:,1),Xdata_d11(:,2),3);% order=3
% fprintf("d11(u)=(%.3f)u3+(%.3f)u2+(%.3f)u+(%.3f)\n",X_coef_d11)
Ydata_d22=importdata("Yforce.txt"); %Damping in sway
Ydata_d22(:,2)=Ydata_d22(:,2)-mean(Ydata_d22(:,2)); % move the YForce data to pass the orgin
% plot(Yddata(:,1),Yddata(:,2));
Y_coef_d22=polyfit(Ydata_d22(:,1),Ydata_d22(:,2),3);% order=3
% fprintf("d22(v)=(%.3f)v3+(%.3f)v2+(%.3f)v+(%.3f)\n",Y_coef_d22)
N_coef_d33=[1/80*d_33_c1*length^4 0 1/12*d_33_c2*length^2 0]; %Damping in yaw
% fprintf("d33(r)=(%.3f)r3+(%.3f)r2+(%.3f)r+(%.3f)\n",N_coef_d33)
% quadratic real coefficient polynomials
X_coef_d11_over_u=X_coef_d11(1:3);
Y_coef_d22_over_v=Y_coef_d22(1:3);
N_coef_d33_over_r=N_coef_d33(1:3);


% Resistance
resistance_table = importdata('resistance.txt');
resistance_coefs = polyfit(resistance_table(:,1),resistance_table(:,2),3);
resistance_coefs(4) = 0;


%% DC Motors
% Armature
Ra = 0.2268; % ohms
La = 0.28; % henry
Kt = 1.7629e-02; % Nm/A
Kb = 1.7629e-02; % V/(rad/s) = Nm/A
% Shaft
Jm = 6.24e-04; % kg m2
Cm = 2.909e-05; % Nm/(rad/s)

%% Forces input
time_interval=0.001; % interval 0.001 seconds
time_all=100; % total time 20 seconds
time_Vector = linspace(0,time_all,time_all/time_interval).';
% time_Vector=transpose((0:(time_all/time_interval))*time_interval);

% force_x_Sample=sin(time_Vector/time_all*4*pi);
% force_x_Sample=step_force(1,2,8,10,time_Vector);
force_x_Sample=zeros(time_all/time_interval,1);

% force_y_Sample=zeros(time_all/time_interval,1);
% force_y_Sample=step_force(1,2,8,10,time_Vector);
force_y_Sample=step_force(0.5,10,35,45,time_Vector);

force_r_Sample=zeros(time_all/time_interval,1);
% force_r_Sample=step_force(1,2,8,10,time_Vector);

tau_Sample=[time_Vector force_x_Sample force_y_Sample force_r_Sample];
% plot(time_Vector,force_x_Sample,time_Vector,force_y_Sample,time_Vector,force_r_Sample);
% legend;


%% Initial conditions
V_initial=[0;0;0]; % initial velocity as zeros
Eta_initial=[0;0;0]; % initial position as zeros



%% CALCULATIONS
%% Bound parameters
inertia_zz = 1/20 * mass*(length^2+beam^2);
rigid_body_Mass = diag([mass mass inertia_zz]);
added_Mass = diag([x_u_dot y_v_dot N_r_dot]); % Added Mass sign unclear
matrix_Mass = rigid_body_Mass + added_Mass;


%% Simulation
paramStruct.StartTime="0";
paramStruct.StopTime=string(time_all);
paramStruct.FixedStep=string(time_interval);
out=sim("simulation.slx",paramStruct);



%% OUTPUT
%% Post processing
Eta=squeeze(out.Eta).';
V=squeeze(out.V).';
T_out=out.tout;


%% Plot
title_char=["X","Y","N","u","v","r","x","y","\psi"];

for i=1:3
    subplot(3,3,i);
    plot(tau_Sample(:,1),tau_Sample(:,1+i));
    title(title_char(i));
    subplot(3,3,i+3);
    plot(T_out,V(:,i));
    title(title_char(i+3));
    subplot(3,3,i+6);
    plot(T_out,Eta(:,i));
    title(title_char(i+6));
end

figure;
plot(Eta(:,1),Eta(:,2));