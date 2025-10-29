function f_I = RR2I(f_R, we, t)

R_I2R = [cos(we*t) -sin(we*t) 0;
         sin(we*t) cos(we*t) 0;
         0 0 1];


f_I = R_I2R.' * f_R;

end

