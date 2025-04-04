close all;clear;clc;
load('motor-tune-0325.mat');
title_chr=["Portside","Starboard"];
dcm_plot=figure();
length_t=125;
tout=tout(1:length_t);
RPM_measured_starboard=RPM_measured_starboard(1:length_t,:);
RPM_measured_portside=RPM_measured_portside(1:length_t,:);
subplot(1,2,1)
plot(tout,RPM_measured_portside);
grid on; title("Step Response "+title_chr(1));
legend("Reference Speed","Actual",'Location','best');
xlabel("Time [s]"); ylabel("Speed [rpm]");
subplot(1,2,2)
plot(tout,RPM_measured_starboard);
grid on; title("Step Response "+title_chr(2));
xlabel("Time [s]"); ylabel("Speed [rpm]");
legend("Reference Speed","Actual",'Location','best');
dcm_plot.Position=[100,100,800,400];