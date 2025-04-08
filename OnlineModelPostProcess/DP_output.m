%% Structuralized output for DP
close all; clear; clc; % close all other plots
% load('calmrun-0401-onlydata.mat');
% load('current10p-longitude-0401-onlydata.mat');
load('current10p-side-0401-2-fine-onlydata.mat');
T_out=actual_pos_ned.time;
t_cut_index=min(find(T_out>0.15)); % ignore the first 0.15 seconds
Eta=actual_pos_ned.signals.values;
T_out=T_out(t_cut_index:end);
Eta=Eta(t_cut_index:end,:);
T_Eta_Ref=desired_pos_ned.time;
num=length(T_Eta_Ref);
T_Eta_Ref_mat=zeros(2*num,1);
T_Eta_Ref_mat(1)=T_out(1);
T_Eta_Ref_mat(end)=T_out(end);
Eta_Ref=desired_pos_ned.signals.values;
Eta_Ref_mat=zeros(2*num,3);
for i = 1:num
    Eta_Ref_mat(2*i-1:2*i,:)=repmat(Eta_Ref(i,:),2,1);
end
for i=2:num
    T_Eta_Ref_mat(2*i-2:2*i-1,:)=repmat(T_Eta_Ref(i),2,1);
end
dp_plot=figure();
title_char=["x","y","\psi"];
for i=1:3
    subplot(3,2,2*i-1);
    plot(T_Eta_Ref_mat,Eta_Ref_mat(:,i),T_out,Eta(:,i));
    legend("Setpoint","Actual",'Location','best');
    title(title_char(i)+" (NED)");
    grid on;xlabel("Time [s]");
    xlim([0,T_out(end)]);
end
ax =subplot(3,2,2);
config_string=sprintf("Experiment Condition = 10%% Current (Side)\n");
config_string=config_string+sprintf("Initial Position = (%.2f, %.2f, %.2f)\n",Eta(1,1),Eta(1,2),Eta(1,3));
status_chr=["ON","OFF"];config_chr=["III","V"];
% config_string=sprintf("StartingPoint = (%.2f, %.2f, %.2f)\n",Eta_initial(1),Eta_initial(2),Eta_initial(3));
for i=1:num
    config_string=config_string+sprintf("Setpoint %d = (%.2f, %.2f, %.2f)\n",i,Eta_Ref(i,1),Eta_Ref(i,2),Eta_Ref(i,3));
end
% config_string=config_string+sprintf("InitialVelocity = (%.4f, %.4f, %.2f)\n",V_initial(1),V_initial(2),V_initial(3));
% config_string=config_string+sprintf("CurrentVelocity = (%.4f, %.4f)\n",EN.current_speed(1),EN.current_speed(2));
% config_string=config_string+sprintf("WindSpeed = (%.2f, %.2f)\n",EN.wind_speed(1),EN.wind_speed(2));
config_string=config_string+sprintf("Using Thrust Allocation Config %s\n",config_chr((5-1)/2));
% config_string=config_string+sprintf("Kalman Filter is %s",status_chr(2-kalman.enable_flag));
text(0,0.5,config_string);
set (ax, 'visible', 'off')
subplot(3,2,[4,6]);
legend_chars=repmat("Empty",num,1);
for i=1:num
    m1=find(T_out==T_Eta_Ref_mat(i*2-1));
    m2=find(T_out==T_Eta_Ref_mat(i*2));
    legend_chars(i)=sprintf("Point %d to Point %d",i-1,i);
    plot(Eta(m1:m2,2),Eta(m1:m2,1));hold on;
end
for i=1:num
    scatter(Eta_Ref(i,2),Eta_Ref(i,1),'filled');hold on;
end
scatter(Eta(1,2),Eta(1,1),'filled');
legend(legend_chars,'Location','best');
xlabel('y');ylabel('x');
title("Trace (NED)");axis equal;grid on;
dp_plot.Position=[100,100,900,500];

%% Motor Output
title_chr=["Portside","Starboard"];
dcm_plot=figure();
% tout=T_out;
tout=T_out(1:floor(end/5*2));
length_t=length(tout);
tout=tout(1:length_t);
RPM_measured_starboard=RPM_set_measured_SB(1:length_t,:);
RPM_measured_portside=RPM_set_measured_PS(1:length_t,:);
subplot(1,2,1)
plot(tout,RPM_measured_portside);
grid on; title("Speed Response "+title_chr(1));
legend("Reference Speed","Actual",'Location','best');
xlabel("Time [s]"); ylabel("Speed [rpm]"); xlim([0,tout(end)]);
subplot(1,2,2)
plot(tout,RPM_measured_starboard);
grid on; title("Speed Response "+title_chr(2));
xlabel("Time [s]"); ylabel("Speed [rpm]"); xlim([0,tout(end)]);
legend("Set Speed","Measured",'Location','best');
dcm_plot.Position=[100,100,800,400];