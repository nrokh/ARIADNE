function dydt = double_SDS(t, y, Ft, Fa, ba, ka, ma, bs, ks, mc)

% interpolate Fa
%Fa = interp1(Ft, Fa, t);

if t < 0.0001
    Fa = Fa(find(Ft==0));
else
    Fa = Fa(find(Ft<t+0.0001&Ft>t-0.0001,1));
end

% calculate dydt vector
dydt = [ y(3); ...
         y(4); ...
         (-ba/ma)*y(3) + (ba/ma)*y(4) - (ka/ma)*y(1) + (ka/ma)*y(2); ...
         (ba/mc)*y(3) - ( (ba/mc) + (bs/mc) )*y(4) + (ka/mc)*y(1) - ( (ka/mc) + (ks/mc) ) *y(2) ]...
         + [0; 0; -Fa/ma; Fa/mc]; 
     

         