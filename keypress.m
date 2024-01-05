function keypress(varargin)
global uk

key = get(gcbf,'CurrentKey');

switch key

    case 'uparrow'
        uk(1) = uk(1) + 1;
        if uk(1) > 100
            uk(1) = 100;
        end
    
    case 'downarrow'
        uk(1) = uk(1) - 1;
        if uk(1) < 0
            uk(1) = 0;
        end
     
    case 'leftarrow'
        uk(2) = uk(2) + 0.1;
        if uk(2) > 30*pi/180
            uk(2) = 30*pi/180;
        end

    case 'rightarrow'
        uk(2) = uk(2) - 0.1;
        if uk(2) < -30*pi/180
            uk(2) = -30*pi/180;
        end

end
