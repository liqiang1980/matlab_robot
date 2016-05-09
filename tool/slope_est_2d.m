%using the regression method to estimate the slop and intercept in the 2d
%case
function gama = slope_est_2d(ctc)
plot(ctc(:,1),ctc(:,2));
[k,b,deltay,deltax,atan2_k,k2] = regression_2d(ctc);
sign(atan2_k)
if(sign(atan2_k) == 1 )
    if((deltay>0)&&(deltax>0))
        gama = k2+pi;
    else
        gama = k2;
    end
else
    if((deltay<0)&&(deltax>0))
        gama = k2+pi;
    else
        gama = k2;
    end
end
hold on;
fplot(@(x)k2*x+b, [1 14]);