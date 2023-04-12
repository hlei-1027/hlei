%% stealthy attack design and its detection
clc;
clear;
%% ��ʼ��
% ϵͳ����
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
% Э�������
Q = 0.001*eye(3);
R =0.01*eye(2);
S = [0.0105 0.0003
    0.0003 0.0105];    % ���������źŵ�Э�������
P = 10*eye(3);  % ����kalman
P_ = P;

K = [0.1643 0.0616
    -0.1475 -0.0805
    0.0276 -0.1826];  % �ȶ���kalman filter gain

L = [0.8681 -0.8864 -0.4226 -0.6788 -0.2934
    0.6013 -1.1476 -0.1093 -1.0402 0.0259];

u = zeros(ny, N);     % �޹��������ź�
ua = zeros(ny, N);    %  ����״̬�µĿ����ź�
ua_ = zeros(ny, N);   % ua_ = ua + beta(k)  ���ܹ�����Ŀ����ź�

x = zeros(nx, N);     % ϵͳ״̬
xa = zeros(nx, N);
x_bar = zeros(nx, N);  % ϵͳ״̬����״̬
xa_bar = zeros(nx, N);
x(:,1) = [2 2 2]';       % ��ʼ��
xa(:,1) = [2 2 2]';      % ��ʼ��
x_bar(:,1) = [0 0 0]';
xa_bar(:,1) = [0 0 0]';
% det_u = -L*

w = mvnrnd([0 0 0], Q, N)';   % ����
v = mvnrnd([0 0], R, N)';
xi = mvnrnd([0 0], S, N)';  % �����ź��еĵ���������

y = zeros(ny, N);           % ϵͳ���
y_bar = C*x_bar;
ya = zeros(ny, N);          % ���ܹ���ʱϵͳ���
ya_bar = C*xa_bar;
ya_ = zeros(ny, N);         % ya_ = ya + arf(k) ������������ź�

% alpha = zeros(ny, N);  % ���������Ĺ����ź�
% alpha  = -ya + ya_bar + xi;
ya_ = ya_bar + xi;
y(:,1) = C * x(:,1) + v(:,1);
ya(:,1) = C * xa(:,1) + v(:,1);

r = [30; 20];                 % �ο�����
e = zeros(ny, N);      % �������
ea = zeros(ny,N);
ea_ = zeros(ny, N);
e(:,1) = r - y(:,1);   % �޹���
ea(:,1) = r -ya(:,1);
ea_(:,1) = r -ya_(:,1);
xe_bar = [x_bar' e']';
xea_bar = [xa_bar' ea_']';
u(:,1) = -L*xe_bar(:,1);   % ��ʼ��
ua(:,1) = -L*xea_bar(:,1);  % ����ʱ�����������ʼ��
beta_ = zeros(ny,N);
beta_(:,100) = [10 10]';   % ע��Ĺ����ź�beta(k)
ua_ = ua + beta_;

F_stable = [0.2119 -0.0699
            -1.7648 -0.0168];
F_unstable = [0.3188 -0.4336
            -1.0578 0.3426];

%% attack detection ����
omiga_k = generate_invmat(ny, N);  % �����޸����ݵ�ʱ��������
psai_k = generate_invmat(ny, N); 

%%  Simulation
for k = 2 : N
    x(:, k) = A * x(:, k-1) + B * u(:,k-1) + w(:,k-1);  % �޹���
    y(:, k) = C * x(:, k) + v(:, k);

    xa(:, k) = A*xa(:,k-1) + B*ua_(:,k-1) + w(:,k-1);  % �й���
    ya(:, k) = C * xa(:, k) + v(:, k);

    %     P_ = A * P * A' + Q;
    %     K = P_ * C'/(C*P*C' + R);
    x_bar(:, k) =  A * x_bar(:, k-1) + B * u(:,k-1) + K*(y(:, k) - C*(A * x_bar(:, k-1) + B * u(:,k-1)));
    y_bar(:,k) = C * x_bar(:,k);
    xa_bar(:, k) =  A * xa_bar(:, k-1) + B * ua(:,k-1) + K*xi(:,k);   % ������Ƶ�kalman �����滻������������
    ya_bar(:,k) = C * xa_bar(:,k);
    ya_(:,k) = ya_bar(:,k) + xi(:,k);

    e(:, k) = r - y(:, k);
    ea_(:,k) = r - ya_(:,k);
    % ����
    xe_bar(:,k) = [(x_bar(:, k) -  x_bar(:, k - 1))' e(:, k)'];
    xea_bar(:,k) = [(xa_bar(:, k) -  xa_bar(:, k - 1))' ea_(:, k)'];

    u(:, k) = u(:, k -1) -L*xe_bar(:,k);
    ua(:, k) = ua(:, k -1) -L*xea_bar(:,k);
    beta_(:,k) = F_stable*beta_(:,k-1);  % ע��Ĺ����ź�
    if k == 100
        beta_(:,k) = [10,10]';   % �����źų�ʼ��
    end

    ua_(:,k) = ua(:,k) + beta_(:,k);
    %     P = (eye(3)-K*C)*P_;
end
%% plot -- �в�һ����
figure(1)
subplot(2,1,1)
z = y - y_bar;       % �в�
plot(t,z);
legend('z_1','z_2')
title("without attack")
subplot(2,1,2)
za = ya_ - ya_bar;   % ����ʱ�в�
plot(t,za);
legend('za_1','za_2')
title("under attack")

%% plot -- �������ƫ���֤������������
figure(2)
subplot(2,1,1)

plot(t,r.*ones(ny,N));
hold on
plot(t,y);  % �޹���ʱ�����
legend('y_1','y_2','r_1','r_2')
title("without attack")

subplot(2,1,2)
plot(t,ya_)  % ������ʱ�����
hold on
plot(t,r.*ones(ny,N));
legend('ya_1''','ya_2''','r_1','r_2')
title("under attack")

figure(3)
plot(t,r.*ones(ny,N));
hold on
plot(t,ya)  % ʵ�ʵ�ϵͳ���
legend('r_1', 'r_2', 'ya_1','ya_2');
%%
