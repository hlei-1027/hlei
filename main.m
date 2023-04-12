%% stealthy attack design and its detection
clc;
clear;
%% 初始化
% 系统矩阵
nx = 3;
ny = 2;
Ts = 0.05;
t = 0: Ts :15;
N = length(t);
A = [0.5091 -0.5592 0.4238
    -0.4267 0.4086 0.5447
    -0.3318 -0.3209 0.9636];

B = [0.6144 -0.2381
    -0.5935 0.0421
    -0.6796 0.5284];

C = [0.5203 -0.6579 0.2095
    0.5783 0.1060 -1.1781];
D = 0;

Ae = [ A    zeros(nx, ny)
    -C*A    eye(ny)];
Be = [B; -C*B];
We = [eye(nx); -C];
Ve = [zeros(nx,ny); eye(ny)];
% 协方差矩阵
Q = 0.001*eye(3);
R =0.01*eye(2);
S = [0.0105 0.0003
    0.0003 0.0105];    % 产生攻击信号的协方差矩阵
P = 10*eye(3);  % 用作kalman
P_ = P;

K = [0.1643 0.0616
    -0.1475 -0.0805
    0.0276 -0.1826];  % 稳定的kalman filter gain

L = [0.8681 -0.8864 -0.4226 -0.6788 -0.2934
    0.6013 -1.1476 -0.1093 -1.0402 0.0259];

u = zeros(ny, N);     % 无攻击控制信号
ua = zeros(ny, N);    %  攻击状态下的控制信号
ua_ = zeros(ny, N);   % ua_ = ua + beta(k)  遭受攻击后的控制信号

x = zeros(nx, N);     % 系统状态
xa = zeros(nx, N);
x_bar = zeros(nx, N);  % 系统状态估计状态
xa_bar = zeros(nx, N);
x(:,1) = [2 2 2]';       % 初始化
xa(:,1) = [2 2 2]';      % 初始化
x_bar(:,1) = [0 0 0]';
xa_bar(:,1) = [0 0 0]';
% det_u = -L*

w = mvnrnd([0 0 0], Q, N)';   % 噪声
v = mvnrnd([0 0], R, N)';
xi = mvnrnd([0 0], S, N)';  % 攻击信号中的第三个部分

y = zeros(ny, N);           % 系统输出
y_bar = C*x_bar;
ya = zeros(ny, N);          % 遭受攻击时系统输出
ya_bar = C*xa_bar;
ya_ = zeros(ny, N);         % ya_ = ya + arf(k) 被攻击的输出信号

% alpha = zeros(ny, N);  % 攻传感器的攻击信号
% alpha  = -ya + ya_bar + xi;
ya_ = ya_bar + xi;
y(:,1) = C * x(:,1) + v(:,1);
ya(:,1) = C * xa(:,1) + v(:,1);

r = [30; 20];                 % 参考输入
e = zeros(ny, N);      % 跟踪误差
ea = zeros(ny,N);
ea_ = zeros(ny, N);
e(:,1) = r - y(:,1);   % 无攻击
ea(:,1) = r -ya(:,1);
ea_(:,1) = r -ya_(:,1);
xe_bar = [x_bar' e']';
xea_bar = [xa_bar' ea_']';
u(:,1) = -L*xe_bar(:,1);   % 初始化
ua(:,1) = -L*xea_bar(:,1);  % 攻击时控制器输出初始化
beta_ = zeros(ny,N);
beta_(:,100) = [10 10]';   % 注入的攻击信号beta(k)
ua_ = ua + beta_;

F_stable = [0.2119 -0.0699
            -1.7648 -0.0168];
F_unstable = [0.3188 -0.4336
            -1.0578 0.3426];

%% attack detection 参数
omiga_k = generate_invmat(ny, N);  % 主动修改数据的时变可逆矩阵
psai_k = generate_invmat(ny, N); 

%%  Simulation
for k = 2 : N
    x(:, k) = A * x(:, k-1) + B * u(:,k-1) + w(:,k-1);  % 无攻击
    y(:, k) = C * x(:, k) + v(:, k);

    xa(:, k) = A*xa(:,k-1) + B*ua_(:,k-1) + w(:,k-1);  % 有攻击
    ya(:, k) = C * xa(:, k) + v(:, k);

    %     P_ = A * P * A' + Q;
    %     K = P_ * C'/(C*P*C' + R);
    x_bar(:, k) =  A * x_bar(:, k-1) + B * u(:,k-1) + K*(y(:, k) - C*(A * x_bar(:, k-1) + B * u(:,k-1)));
    y_bar(:,k) = C * x_bar(:,k);
    xa_bar(:, k) =  A * xa_bar(:, k-1) + B * ua(:,k-1) + K*xi(:,k);   % 攻击设计的kalman 用于替换攻击后的输出，
    ya_bar(:,k) = C * xa_bar(:,k);
    ya_(:,k) = ya_bar(:,k) + xi(:,k);

    e(:, k) = r - y(:, k);
    ea_(:,k) = r - ya_(:,k);
    % 更新
    xe_bar(:,k) = [(x_bar(:, k) -  x_bar(:, k - 1))' e(:, k)'];
    xea_bar(:,k) = [(xa_bar(:, k) -  xa_bar(:, k - 1))' ea_(:, k)'];

    u(:, k) = u(:, k -1) -L*xe_bar(:,k);
    ua(:, k) = ua(:, k -1) -L*xea_bar(:,k);
    beta_(:,k) = F_stable*beta_(:,k-1);  % 注入的攻击信号
    if k == 100
        beta_(:,k) = [10,10]';   % 攻击信号初始化
    end

    ua_(:,k) = ua(:,k) + beta_(:,k);
    %     P = (eye(3)-K*C)*P_;
end
%% plot -- 残差一致性
figure(1)
subplot(2,1,1)
z = y - y_bar;       % 残差
plot(t,z);
legend('z_1','z_2')
title("without attack")
subplot(2,1,2)
za = ya_ - ya_bar;   % 攻击时残差
plot(t,za);
legend('za_1','za_2')
title("under attack")

%% plot -- 输出发生偏差，验证攻击的隐蔽性
figure(2)
subplot(2,1,1)

plot(t,r.*ones(ny,N));
hold on
plot(t,y);  % 无攻击时的输出
legend('y_1','y_2','r_1','r_2')
title("without attack")

subplot(2,1,2)
plot(t,ya_)  % 被攻击时的输出
hold on
plot(t,r.*ones(ny,N));
legend('ya_1''','ya_2''','r_1','r_2')
title("under attack")

figure(3)
plot(t,r.*ones(ny,N));
hold on
plot(t,ya)  % 实际的系统输出
legend('r_1', 'r_2', 'ya_1','ya_2');
%%
