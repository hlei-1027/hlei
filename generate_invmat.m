%% 生成 kmax个可逆的矩阵
function Ak =  generate_invmat(n,kmax)
    % 创建空的可逆矩阵数组
Ak = zeros(2,2,kmax);

% 循环生成每个时间步的可逆矩阵
for k = 1:kmax
    % 生成随机矩阵
    M = rand(n,n);
    
    % 计算行列式
    det_M = det(M);
    
    % 如果行列式为0，则重新生成随机矩阵
    while det_M == 0
        M = rand(n,n);
        det_M = det(M);
    end
    
    % 计算逆矩阵
    Ak(:,:,k) = inv(M);
end

end
