%% 判断系统是否观

function observe(A, B, C,D)
    % 定义系统状态空间模型
    sys = ss(A,B,C,D);

    % 计算可观测性矩阵
    O = obsv(sys);

    % 判断系统是否可观测
    if rank(O) == size(A,1)
        disp('The system is observable.');
    else
        disp('The system is not observable.');
    end
end