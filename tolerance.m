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
    
    distance = sqrt((xf - xe)^2+ (yf - ye)^2 + (zf - ze)^2);

%     distance = norm(dummy, 2);
    
    if distance <= epsi
        isRange = true;
        disp(distance)
    else
        isRange = false;
    end
 
end



