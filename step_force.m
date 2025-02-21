function force = step_force(max_force,t1,t2,t3,t)
force=zeros(length(t),1);
for i = 1:length(t)
    if t(i)<t1
        force(i)=max_force*(t(i)/t1);
    elseif t(i)<t2
        force(i)=max_force;
    elseif t(i)<t3
        force(i)=max_force*(t3-t(i))/(t3-t2);
    end
end

end