%% Tolerance 


function [isRange, distance] = tolerance(friendly, enemy, l)
    epsi = l/2;
    xf = friendly(1);
    yf = friendly(2);
    zf = friendly(3);
    xe = enemy(1);
    ye = enemy(2);
    ze = enemy(3);
    dummy = [friendly, enemy];
    
%     distance = sqrt((xf^2 - xe^2)+ (yf^2 - ye^2) + (zf^2 - ze^2));

    distance = norm(dummy, 2);
    
    if distance <= epsi
        isRange = true;
    else
        isRange = false
    end
 
end



