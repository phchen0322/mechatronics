%% Structuralized output for DP
close all; % close all other plots
dp_plot=figure();
subplot(3,3,3);
yyaxis left;plot(T_out,out.bat_out(:,1));grid on;
yyaxis right;plot(T_out,out.bat_out(:,2));grid on;
title('Voltage & Current');legend("Voltage/V","Current/A");
subplot(3,3,6);
plot(T_out,out.bat_out(:,3));title("SoC");grid on;
subplot(3,3,9);grid on;
yyaxis left;plot(T_out,out.power_out(:,1));grid on;
yyaxis right;plot(T_out,out.power_out(:,2));grid on;
title(['Power & Energy with Config', num2str(TA.config)]);legend("Power/W","Energy/J");
title_char=["x","y","\psi"];
for i=1:3
    subplot(3,3,3*i-2);
    plot([0,T_out(end)],[Eta_Ref(i),Eta_Ref(i)],T_out,Eta(:,i));
    legend("Setpoint","Actual",'Location','best');
    title(title_char(i)+" (NED)");
    grid on;
end
ax =subplot(3,3,2);status_chr=["ON","OFF"];config_chr=["III","V"];
config_string=sprintf("StartingPoint = (%.2f, %.2f, %.2f)\n",Eta_initial(1),Eta_initial(2),Eta_initial(3));
config_string=config_string+sprintf("Setpoint = (%.2f, %.2f, %.2f)\n",Eta_Ref(1),Eta_Ref(2),Eta_Ref(3));
config_string=config_string+sprintf("InitialVelocity = (%.4f, %.4f, %.2f)\n",V_initial(1),V_initial(2),V_initial(3));
config_string=config_string+sprintf("CurrentVelocity = (%.4f, %.4f)\n",EN.current_speed(1),EN.current_speed(2));
config_string=config_string+sprintf("WindSpeed = (%.2f, %.2f)\n",EN.wind_speed(1),EN.wind_speed(2));
config_string=config_string+sprintf("Using Thrust Allocation Config %s\n",config_chr((TA.config-1)/2));
config_string=config_string+sprintf("Kalman Filter is %s",status_chr(2-kalman.enable_flag));
text(0,0.5,config_string);
set (ax, 'visible', 'off')
subplot(3,3,[5,8]);
plot(Eta(:,2),Eta(:,1));xlabel('y');ylabel('x');
title("Trace (NED)");axis equal;grid on;
dp_plot.Position=[100,100,1000,400];