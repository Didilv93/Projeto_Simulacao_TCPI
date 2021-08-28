function [freq] = getSysFrequency(sys)
    [~,initcross] = dutycycle(sys);
    if(sum(sys) == 0)
        freq = 0;
    else
        v_aux = initcross / 10000;
        if(sum(sys) > 0)
            direction = 1;
        else
            direction = -1;
        end
        freq = direction/(v_aux(2) - v_aux(1));
    end
end