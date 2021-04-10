function isTrue = insideAirspace(enemy, timeSpan)

 xe = enemy(1);
 ye = enemy(2);
 ze = enemy(3);
 

 
 for i = timeSpan
     
    if ((xe < -5) || (xe > 5))
        disp('Enemy has left the airspace')
        isTrue = false; 
    elseif ((ye < -5) || (ye > 5))
        disp('Enemy has left the airspace')
        isTrue = false;
    elseif((ze < 0) || (ze > 10))
        disp('Enemy has left the airspace')
        isTrue = false;
    else
        disp('Enemy is in the air Space')
        isTrue = true;
    end


end 
