function [y] = thrust_bow_sign(p,x)
%   calculate the value of polyval(p1 or p2 ,x) but the sign is decided by the sign
%   of x, if x>0 then p1,x; if x<0 then p2,x; also ensure for x>0 y>=0; x<0,  y<=0
%   good for calculating generated force by thruster
y=zeros(length(x),1);
for i=1:length(x)
    if(x(i)<0)
        y(i)=-polyval(p,-x(i));
        if y(i)>0
          y(i)=0;
        end
    elseif (x(i)>0)
        y(i)=polyval(p,x(i));
        if y(i)<0
            y(i)=0;
        end
    else
        y(i)=0;
    end
end