% G distance
function d = cal_G(a,b)
global Gridsize;
   if ( (abs(a(1) - b(1)) == Gridsize) && (abs(a(2) - b(2)) == Gridsize ) )
     d = 1.4*Gridsize;
   else
     d = 1*Gridsize;
    end
end