close all;clear;clc;
% off_data=zeros(4);
off_data(1)=load('offline_-15_0.mat');
off_data(2)=load('offline_-15_0.785.mat');
off_data(3)=load('offline_-15_1.57.mat');
off_data(4)=load('offline_-15_3.14.mat');
dp_plot=figure();
title_char=["x","y","\psi"];
for j=1:4
    for i=1:3
        subplot(3,4,j+4*(i-1));
        plot([0,off_data(j).T_out(end)],[off_data(j).Eta_Ref(i),off_data(j).Eta_Ref(i)],off_data(j).T_out,off_data(j).Eta(:,i));
        legend("Setpoint","Actual",'Location','best');
        title(title_char(i));
        grid on;
    end
end
suptitle("Setpoint Expriments (0, \pi/4, \pi/2 and \pi) Under 15% Current (-x Direction)");
% ax =subplot(3,2,2);
% status_chr=["ON","OFF"];config_chr=["III","V"];
% config_string=sprintf("StartingPoint = (%.2f, %.2f, %.2f)\n",Eta_initial(1),Eta_initial(2),Eta_initial(3));
% config_string=config_string+sprintf("Setpoint = (%.2f, %.2f, %.2f)\n",Eta_Ref(1),Eta_Ref(2),Eta_Ref(3));
% config_string=config_string+sprintf("InitialVelocity = (%.4f, %.4f, %.2f)\n",V_initial(1),V_initial(2),V_initial(3));
% config_string=config_string+sprintf("CurrentVelocity = (%.4f, %.4f)\n",EN.current_speed(1),EN.current_speed(2));
% config_string=config_string+sprintf("WindSpeed = (%.2f, %.2f)\n",EN.wind_speed(1),EN.wind_speed(2));
% config_string=config_string+sprintf("Using Thrust Allocation Config %s\n",config_chr((TA.config-1)/2));
% config_string=config_string+sprintf("Kalman Filter is %s",status_chr(2-kalman.enable_flag));
% text(0,0.5,config_string);
% set (ax, 'visible', 'off')
% subplot(3,2,[4,6]);
% plot(Eta(:,2),Eta(:,1));xlabel('y');ylabel('x');
% title("Trace (NED)");axis equal;grid on;
dp_plot.Position=[100,100,1000,600];