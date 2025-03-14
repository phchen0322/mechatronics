clc
clear
close all


%% INPUTS
%% Simulation Timeline
time_interval=0.01; % interval 0.01 seconds
time_all=120; % total time 120 seconds
time_Vector = linspace(0,time_all,time_all/time_interval).';

%% Equations of motion, struct TN (TitoNeri)
% Constants
TN.mass = 18; %Mass
TN.length = 0.97; %Length
TN.beam = 0.32; %Width
TN.x_u_dot=1.2;
TN.y_v_dot=66.1;
TN.N_r_dot=2.3;
TN.M_v_dot=0; TN.Y_r_dot=0;
TN.d_33_c1 = 122.3;
TN.d_33_c2 = 4.8;

% Estimation of d11 d22 d33
Xdata_d11=importdata('Xforce.txt'); %Damping in surge
% X_coef_d11=polyfit(Xdata_d11(:,1),Xdata_d11(:,2),3);% order=3
% fprintf("d11(u)=(%.3f)u3+(%.3f)u2+(%.3f)u+(%.3f)\n",X_coef_d11)
Ydata_d22=importdata("Yforce.txt"); %Damping in sway
% Ydata_d22(:,2)=Ydata_d22(:,2)-mean(Ydata_d22(:,2)); % move the YForce data to pass the orgin
% plot(Yddata(:,1),Yddata(:,2));
% Y_coef_d22=polyfit(Ydata_d22(:,1),Ydata_d22(:,2),3);% order=3
% fprintf("d22(v)=(%.3f)v3+(%.3f)v2+(%.3f)v+(%.3f)\n",Y_coef_d22)
N_coef_d33=[1/80*TN.d_33_c1*TN.length^4 0 1/12*TN.d_33_c2*TN.length^2 0]; %Damping in yaw
% fprintf("d33(r)=(%.3f)r3+(%.3f)r2+(%.3f)r+(%.3f)\n",N_coef_d33)
% quadratic real coefficient polynomials, Note that this is from external
% excel fit 3-order by constraining passing the origin (see XYForce.xlsx)
TN.X_coef_d11_over_u=[17.601,-1.2378,0.6164];
TN.Y_coef_d22_over_v=[121.46,-0.3041,4.8686];
TN.N_coef_d33_over_r=N_coef_d33(1:3);

% Fit check: plot the original curve and the fitted curve in X and Y
fit_damping_plot=figure();
subplot(1,2,1);
plot(Xdata_d11(:,1),Xdata_d11(:,2),Xdata_d11(:,1),polyval([TN.X_coef_d11_over_u,0],Xdata_d11(:,1)));
title('Fit of damping X'); grid on;
subplot(1,2,2);
plot(Ydata_d22(:,1),Ydata_d22(:,2),Ydata_d22(:,1),polyval([TN.Y_coef_d22_over_v,0],Ydata_d22(:,1)));
title('Fit of damping Y'); grid on;
fit_damping_plot.Position=[100,100,1000,600];
% Resistance, not that now it is not used in the model
resistance_table = importdata('resistance.txt');
TN.resistance_coefs = polyfit(resistance_table(:,1),resistance_table(:,2),3);
TN.resistance_coefs(4) = 0;

%% DC Motors, struct DC
% Armature
DC.Ra = 0.2268; % ohms
DC.La = 0.28; % henry
DC.Kt = 1.7629e-02; % Nm/A
DC.Kb = 1.7629e-02; % V/(rad/s) = Nm/A
% Shaft
DC.Jm = 6.24e-04; % kg m2
DC.Cm = 2.909e-05; % Nm/(rad/s)
% Other parameters used for control
DC.wrated=564.9; %rated speed, rad/s;
DC.irated=9;%rated current, A
DC.vrated=12;%rated voltage, V

%% DC Motor Controller, struct DCon
DCon.load_const=5.85E-7; % before properller model finished, for now simply 
% use a load constant to consider the mechanical torque led by rotation speed
% should be deleted after completion of propeller model
DCon.Kp=10;
DCon.Ki=1.2;
DCon.Kd=0.1;
DCon.N=100; %4 parameters for a PID controller
% w_ref is only for tunning, should be commented in further usage

%% Speed Setting input (just for PID experiment)
DCon.w_ref=[0,0;15,500/60*pi*2;25,500/60*pi*2;40,300/60*pi*2;41,700/60*pi*2;...
    60,700/60*pi*2;75,900/60*pi*2;90,900/60*pi*2;time_all,700/60*pi*2];
%DCon.w_ref=[0,900/60*pi*2;time_all,900/60*pi*2];

%% Forces input (just for expriment)

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
% For the vessel Tito Neri (TN)
TN.inertia_zz = 1/20 * TN.mass*(TN.length^2+TN.beam^2);
TN.rigid_body_Mass = diag([TN.mass TN.mass TN.inertia_zz]);
TN.added_Mass = diag([TN.x_u_dot TN.y_v_dot TN.N_r_dot]); % Added Mass sign unclear
TN.matrix_Mass = TN.rigid_body_Mass + TN.added_Mass;


%% Simulation
paramStruct.StartTime="0";
paramStruct.StopTime=string(time_all);
paramStruct.FixedStep=string(time_interval);
out=sim("simulation.slx",paramStruct);

%% OUTPUT
T_out=out.tout;
%% Post processing and Plot for PID Control
W_dcon1_ref=out.dcon_out(:,1);
W_dcon1=out.dcon_out(:,2);
dcon_performance_plot=figure();
subplot(1,2,1);plot(T_out,W_dcon1_ref,T_out,W_dcon1);
legend("Set Speed","Speed",'Location', 'northwest'); title('Controller Performance');
subplot(1,2,2);plot(T_out,(W_dcon1_ref-W_dcon1)./W_dcon1_ref,[0,time_all],[0.02,0.02]);
legend("Error Percentage","2% Margin"); title("Error");
dcon_performance_plot.Position=[100,100,1000,600];

%% Post processing for equation motion
Eta=squeeze(out.Eta).';
V=squeeze(out.V).';

%% Plot for equation motion
title_char=["X","Y","N","u","v","r","x","y","\psi"];
figure();
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