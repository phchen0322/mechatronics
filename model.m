clc
clear
close all


%% INPUTS
%% Simulation Timeline
time_interval=0.01; % interval 0.01 seconds
time_all=200; % total time 200 seconds
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
plot(Xdata_d11(:,1),Xdata_d11(:,2),'*',Xdata_d11(:,1),polyval([TN.X_coef_d11_over_u,0],Xdata_d11(:,1)));
legend("Measured","Fit");xlabel("Velocity [m/s]");ylabel("Force [N]");
title('Fit of damping X'); grid on;
subplot(1,2,2);
plot(Ydata_d22(:,1),Ydata_d22(:,2),'*',Ydata_d22(:,1),polyval([TN.Y_coef_d22_over_v,0],Ydata_d22(:,1)));
legend("Measured","Fit");xlabel("Velocity [m/s]");ylabel("Force [N]");
title('Fit of damping Y'); grid on;
fit_damping_plot.Position=[100,100,1000,600];
% Resistance, not that now it is not used in the model
resistance_table = importdata('resistance.txt');
TN.resistance_coefs = polyfit(resistance_table(:,1),resistance_table(:,2),3);
TN.resistance_coefs(4) = 0;

% For the vessel Tito Neri (TN)
TN.inertia_zz = 1/20 * TN.mass*(TN.length^2+TN.beam^2);
TN.rigid_body_Mass = diag([TN.mass TN.mass TN.inertia_zz]);
TN.added_Mass = diag([TN.x_u_dot TN.y_v_dot TN.N_r_dot]); % Added Mass sign positive
TN.matrix_Mass = TN.rigid_body_Mass + TN.added_Mass;

%% Environmental Forces, struct EN
% Current
EN.current_speed=[0 0];%(in NED frame, only X and Y)
EN.current_v_Sample=[time_Vector repmat(EN.current_speed,time_all/time_interval,1)];

% Wind
TN.height_above_water=0.20;%suppose 20cm above water for wind calculation
EN.wind_AFW=pi*TN.beam*TN.height_above_water/4;
EN.wind_ALW=pi*TN.length*TN.height_above_water/4;
EN.wind_Loa=TN.length;%area (half ellipse) and length
EN.wind_config=[EN.wind_AFW;EN.wind_ALW;EN.wind_AFW*EN.wind_Loa];
EN.wind_speed=[0 0];%(NED, X and Y)
EN.wind_v_Sample=[time_Vector repmat(EN.wind_speed,time_all/time_interval,1)];%NED


%% Thrust Allocation, struct TA
% for experiment pls change TA.config to 3 or 5
TA.config3_T=[1      0     1      0    0;
              0      1     0      1    1;
              0.065 -0.35 -0.065 -0.35 0.35];
TA.config5_T=[0,1,0;1,0,1;-0.35,-0.065,0.35];
TA.config5_T_inv=inv(TA.config5_T);
TA.config=3; % the chosen config
thrust_Raw=load('data_Thrust.mat');
% ps data: rpm=501.15 \sqrt(thrust)+186.43
%TA.poly_thrust_ps1=[3.540E-6,-7.340E-4,0];
%TA.poly_thrust_ps2=[3.263E-6,-9.771E-4,0];
% TA.poly_thrust_ps=[511.95,137.04];
% TA.poly_thrust_sb=[602.77,145.38];
% TA.poly_thrust_bow=[0.2934,0.1577];
% Note that in the wetmodel dc motors are 608.58, 200
% Note that in the wetmodel bow thruster is 0.13 0.17 (*0.8)

% plot_thrust_fit=figure();
% subplot(1,3,1);
% plot(thrust_Raw.data_Thrust_ps(:,1),thrust_Raw.data_Thrust_ps(:,2), 'o',...
%     thrust_cal_sign(TA.poly_thrust_ps,-3:0.05:3),[-3:0.05:3])
% subplot(1,3,2);
% plot(thrust_Raw.data_Thrust_sb(:,1),thrust_Raw.data_Thrust_sb(:,2), 'o',...
%     thrust_cal_sign(TA.poly_thrust_sb,-3:0.05:3),[-3:0.05:3])
% subplot(1,3,3);
% plot(thrust_Raw.data_Thrust_bow(:,1),thrust_Raw.data_Thrust_bow(:,2), 'o',...
%     thrust_bow_sign(TA.poly_thrust_bow,[-2:0.05:2]),[-2:0.05:2])
% set(plot_thrust_fit,'position',[100,100,1000,600])


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

%% Shafts
shaft.inertia1 = 1.974e-5; % kg m2
shaft.inertia2 = 2.06293e-6; % kg m2
shaft.inertia3 = 1.96350e-8; % kg m2

shaft.frictionTranslatedTo1 = (1 - 0.94 * 0.95 * 0.96 * 0.95^2) * 0.02573 / 648; % kg m2/s

shaft.gearRatio1to2 = 1/3;
shaft.gearRatio2to3 = 1;

shaft.fullInertiaTranslatedTo1 = DC.Jm + shaft.inertia1 + shaft.inertia2 * shaft.gearRatio1to2^2 + shaft.inertia3 * shaft.gearRatio1to2^2 * shaft.gearRatio2to3^2;
shaft.fullFrictionTranslatedTo1 = DC.Cm + shaft.frictionTranslatedTo1;

%% Propellers
propeller.diameter = 0.065;
propeller.waterDensity = 1000;
KTCurve = importdata('KT.txt');
KQCurve = importdata('KQ.txt');
propeller.JForKT = KTCurve(:,1);
propeller.KT = KTCurve(:,2);
propeller.JForKQ = KQCurve(:,1);
propeller.KQ = KQCurve(:,2);


%% DC Motor Controller, struct DCon
DCon.load_const=5.85E-7; % before properller model finished, for now simply 
% use a load constant to consider the mechanical torque led by rotation speed
% should be deleted after completion of propeller model
DCon.Kp=80;
DCon.Ki=10.8;
DCon.Kd=1;
DCon.N=100; %4 parameters for a PID controller
% w_ref is only for tunning, should be commented in further usage

%% Speed Setting input (just for PID experiment)
DCon.w_ref=[0,0;15,500/60*pi*2;25,500/60*pi*2;40,300/60*pi*2;41,700/60*pi*2;...
   60,700/60*pi*2;75,900/60*pi*2;90,900/60*pi*2;time_all,700/60*pi*2];
% DCon.w_ref=[0,900/60*pi*2;time_all,900/60*pi*2];

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

%% Thrust Simulatior, simulating the output thrust by acuators, structure TSim
% based on the input rpm, andgle and gain, calculate the tau (BODY frame)
% interpolation of the curve thrust to speed, but add (0,0)
% thrust_Raw=load('data_Thrust.mat');
% TSim.thrust_ps_rpm=thrust_Raw.data_Thrust_ps;
% TSim.thrust_ps_rpm=[TSim.thrust_ps_rpm(1:8,:);0,0;TSim.thrust_ps_rpm(9:end,:)];
% TSim.thrust_sb_rpm=thrust_Raw.data_Thrust_sb;
% TSim.thrust_sb_rpm=[TSim.thrust_sb_rpm(1:8,:);0,0;TSim.thrust_sb_rpm(9:end,:)];
% TSim.thrust_bow_gain=thrust_Raw.data_Thrust_bow;
% TSim.thrust_bow_gain(8,:)=[0,0];
% -0.5 for -90degree +0.5 for -90 degree, use config 3
TSim.config3_T=TA.config3_T;
speed_ps_angle_Sample=0*ones(time_all/time_interval,1);
speed_sb_angle_Sample=0*ones(time_all/time_interval,1);
speed_ps_rpm_Sample=0*ones(time_all/time_interval,1);
speed_sb_rpm_Sample=0*ones(time_all/time_interval,1);
speed_bow_gain_Sample=0*ones(time_all/time_interval,1);
TSim.speed_Sampe=[time_Vector,speed_ps_angle_Sample,speed_sb_angle_Sample,...
    speed_ps_rpm_Sample,speed_sb_rpm_Sample,speed_bow_gain_Sample];

%% PID Controllers for calculate required forces from position error
% struct PCon
PCon.KX=[0.005;0;0.01;100]; %P, I, D and N
PCon.KY=[0.1;0;0.1;100]; %P, I, D and N
PCon.KN=[0.1;0;0;100]; %P, I, D and N
% PCon.KX=[0.006673;0.0004631;2.1557;12.6]; %P, I, D and N
% PCon.KY=[0.1697;0.0007710;4.7815;6.41]; %P, I, D and N
% PCon.KN=[0.14737;0.004640;1.0114;1.7889]; %P, I, D and N

%% Battery, struct BAT
BAT=load('BatParameters.mat').Bat;
BAT.power_others=5; % suppose other components consumes 5W in total on average
BAT.converter_efficiency=0.95; % suppose the buck-boost converter has an efficiency of 95%

%% Kalman filter
kalman.enable_flag=0; % 0 for disable kalman and 1 for enable
kalman.TSampling = 0.1;
unknownForcesExpectedAmplitude = diag([1.2 1.2 1.2]);
inverseFullMass = eye(3) / (TN.rigid_body_Mass + TN.added_Mass);

kalman.Q = [zeros(3,3) , zeros(3,3) ; zeros(3,3) , (inverseFullMass * unknownForcesExpectedAmplitude * kalman.TSampling)^2];
kalman.R = diag([0.06^2 0.06^2 0.009^2]);
kalman.H = [eye(3) zeros(3,3)];

targetOvershoot = 0.05;
targetRiseTime = 20;
kalman.KDiscreteIntegrators = diag([0.026 0.01 0.01]);

kalman.sqrtR = chol(kalman.R);

C0 = zeros(3,3);
d110 = polyval(TN.X_coef_d11_over_u,0);
d220 = polyval(TN.Y_coef_d22_over_v,0);
d330 = polyval(TN.N_coef_d33_over_r,0);
D0 = diag([d110 d220 d330]);
kalman.continuousStateTransitionAround0 = [zeros(3,3) eye(3) ; zeros(3,3) -inverseFullMass*(C0 + D0)];
kalman.continuousStateInputTransitionAround0 = [zeros(3,3) ; inverseFullMass];

continuousLinearizationAround0 = ss(kalman.continuousStateTransitionAround0, kalman.continuousStateInputTransitionAround0, kalman.H, zeros(3,3));
discreteLinearizationAround0 = c2d(continuousLinearizationAround0, kalman.TSampling, 'zoh');

kalman.stateTransition = discreteLinearizationAround0.A;
kalman.inputStateTransition = discreteLinearizationAround0.B;

targetDampingRatio = -log(targetOvershoot)/sqrt(pi^2 + log(targetOvershoot)^2);
targetUndampedFrequency = 1/(targetRiseTime * sqrt(1 - targetDampingRatio^2)) * (pi - atan(sqrt(1 - targetDampingRatio^2)/targetDampingRatio));

dominantPole1 = (-targetDampingRatio + sqrt(targetDampingRatio^2 - 1))*targetUndampedFrequency;
dominantPole2 = (-targetDampingRatio - sqrt(targetDampingRatio^2 - 1))*targetUndampedFrequency;

sPoles = [dominantPole1 dominantPole2 -0.1 -0.2 -0.3 -0.4];
zPoles = exp(sPoles * kalman.TSampling);

kalman.discreteControllerGains = place(discreteLinearizationAround0.A, discreteLinearizationAround0.B, zPoles);

%% Initial conditions
V_initial=[0;0;0]; % initial velocity as zeros
% V_initial=[0;0;0]+[EN.current_speed';0]; % initial velocity as zeros related to current
Eta_initial=[0;0;0]; % initial position as zeros

%% Set conditions
Eta_Ref=[0;0;0];
% Eta_Ref=[0.5;-0.8;0.4];

%% Setting the important parameters together
TA.config=3; % Choosing thrust allocation configuration
kalman.enable_flag=0; % enable or disable kalman filter in closed loop
EN.current_speed=[-0.0*0.15 0*0.15]; % Set current speed(in NED frame, only X and Y)
EN.current_v_Sample=[time_Vector repmat(EN.current_speed,time_all/time_interval,1)];
V_initial=[0;0;0]+[EN.current_speed';0]; % initial velocity as zeros related to current
EN.wind_speed=[0 0]; % Set wind speed (in NED frame, only X and Y)
EN.wind_v_Sample=[time_Vector repmat(EN.wind_speed,time_all/time_interval,1)];%NED
Eta_Ref=[1;0;pi/2]; % desired set point

%% Simulation
paramStruct.StartTime="0";
paramStruct.StopTime=string(time_all);
paramStruct.FixedStep=string(time_interval);
out=sim("simulation.slx",paramStruct);

%% OUTPUT
T_out=out.tout;

%% Output and plot for Thrust Allocation
plot_thrust_alloc=figure();
set(plot_thrust_alloc,'position',[100,100,1400,600])
subplot(1,4,1);
plot(out.tout,out.force_required(:,1),out.tout,out.force_required(:,2),out.tout,out.force_required(:,3));
legend("Force X","Force Y","Force N");
subplot(1,4,2);
plot(out.tout,out.speed_ref(:,1),out.tout,out.speed_ref(:,2));
legend("Speed PS", "Speed SB");
subplot(1,4,3);
plot(out.tout,out.angle_ref(:,1),out.tout,out.angle_ref(:,2));
legend("Angle PS","Angle SB");
subplot(1,4,4);
plot(out.tout,out.speed_ref(:,3));
legend("Gain Bow Thruster");

%% Post processing and Plot for PID Control
W_dcon1_ref=out.dcon_out(:,1);
W_dcon1=out.dcon_out(:,2);
dcon_performance_plot=figure();
subplot(1,2,1);plot(T_out,W_dcon1_ref,T_out,W_dcon1);
ylabel("Speed [rad/s]"); xlabel("Time [s]");grid on;
legend("Set Speed","Speed",'Location', 'best'); title('DC Controller Starboard');
subplot(1,2,2);plot(T_out,100.*(W_dcon1_ref-W_dcon1)./W_dcon1_ref,[0,time_all],100.*[0.02,0.02],[0,time_all],100.*[-0.02,-0.02]);
legend("Relative Error","2% Margin","-2% Margin"); title("Error Percentage");ytickformat('percentage');
xlabel("Time [s]"); grid on;
dcon_performance_plot.Position=[100,100,800,300];

%% Plot for karlman filter performance
km_Eta_est=squeeze(out.KM_est_out);
km_plot=figure();
title_char=["Estimated","True","Measured"];
for i=1:3
    subplot(1,3,(i+1)-3*floor(i/3));
    plot(T_out,km_Eta_est(i*3-2,:),T_out,km_Eta_est(i*3-1,:),T_out,km_Eta_est(i*3,:));
    title(title_char(i))
    legend('x','y','\psi');
    grid on;
    xlabel("Time [s]");
end
km_plot.Position=[100,100,1000,400];

%% Post processing for equation motion
Eta=squeeze(out.Eta).';
V=squeeze(out.V).';
Tau_all=squeeze(out.Tau_all).';

%% Plot for equation motion
title_char=["X","Y","N","u","v","r","x","y","\psi"];
figure();
for i=1:3
    subplot(3,3,i);
    plot(T_out,Tau_all(:,i));
    title(title_char(i));
    subplot(3,3,i+3);
    plot(T_out,V(:,i));
    title(title_char(i+3));
    subplot(3,3,i+6);
    plot(T_out,Eta(:,i));
    title(title_char(i+6));
end

figure;
plot(Eta(:,2),Eta(:,1));xlabel('y');ylabel('x');axis equal;

%% Plot for battery output
bat_plot=figure();
subplot(1,3,1);
yyaxis left;plot(T_out,out.bat_out(:,1));
yyaxis right;plot(T_out,out.bat_out(:,2));
title('Voltage & Current');legend("Voltage/V","Current/A");
subplot(1,3,2);
plot(T_out,out.bat_out(:,3));title("SoC");
subplot(1,3,3);
yyaxis left;plot(T_out,out.power_out(:,1));
yyaxis right;plot(T_out,out.power_out(:,2));
title(['Power & Energy with Config', num2str(TA.config)]);legend("Power/W","Energy/J");
bat_plot.Position=[100,100,1000,600];

%% Structuralized output for DP
% close all; % close all other plots
dp_plot=figure();
title_char=["x","y","\psi"];
for i=1:3
    subplot(3,2,2*i-1);
    plot([0,T_out(end)],[Eta_Ref(i),Eta_Ref(i)],T_out,Eta(:,i));
    legend("Setpoint","Actual",'Location','best');
    title(title_char(i)+" (NED)");
    grid on;
end
ax =subplot(3,2,2);status_chr=["ON","OFF"];config_chr=["III","V"];
config_string=sprintf("StartingPoint = (%.2f, %.2f, %.2f)\n",Eta_initial(1),Eta_initial(2),Eta_initial(3));
config_string=config_string+sprintf("Setpoint = (%.2f, %.2f, %.2f)\n",Eta_Ref(1),Eta_Ref(2),Eta_Ref(3));
config_string=config_string+sprintf("InitialVelocity = (%.4f, %.4f, %.2f)\n",V_initial(1),V_initial(2),V_initial(3));
config_string=config_string+sprintf("CurrentVelocity = (%.4f, %.4f)\n",EN.current_speed(1),EN.current_speed(2));
config_string=config_string+sprintf("WindSpeed = (%.2f, %.2f)\n",EN.wind_speed(1),EN.wind_speed(2));
config_string=config_string+sprintf("Using Thrust Allocation Config %s\n",config_chr((TA.config-1)/2));
config_string=config_string+sprintf("Kalman Filter is %s",status_chr(2-kalman.enable_flag));
text(0,0.5,config_string);
set (ax, 'visible', 'off')
subplot(3,2,[4,6]);
plot(Eta(:,2),Eta(:,1));xlabel('y');ylabel('x');
title("Trace (NED)");axis equal;grid on;
dp_plot.Position=[100,100,800,500];
% dp_plot_with_bat;
% save(".\OfflineModelOutput\offline_"+num2str(EN.current_speed(1)/0.15*100,2)+"_"+num2str(Eta_Ref(3),3)+".mat","config_string","Eta_Ref","Eta","T_out");