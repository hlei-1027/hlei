%% 判断系统是否可控

function controll(A,C)
O = obsv(A,C);
    if rank(O) == size(A,1)
        disp('The system is detectable.');
    else
        disp('The system is not detectable.');
    end
end
