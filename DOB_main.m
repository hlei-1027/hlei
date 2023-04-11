% import necessary libraries
addpath('scipy');
addpath('matplotlib');
addpath('numpy');
addpath('math');
addpath('scipy/spatial');

% set up matplotlib
is_ipython = contains(matlab.graphics.getBackend, 'MGT');
if is_ipython
    display = @(x) disp('');
    clear_output = @(x) disp('');
end
figure('Position', [0, 0, 1200, 200]);

classdef UGV_model
    properties
        x % X
        y % Y
        theta % heading
        l % wheel base
        v % speed
        dt % decision time periodic
    end
    
    methods
        function obj = UGV_model(x0, y0, theta0, L, v0, T)
            obj.x = x0;
            obj.y = y0;
            obj.theta = theta0;
            obj.l = L;
            obj.v = v0;
            obj.dt = T;
        end
        
        function update(obj, vt, deltat)
            dx = obj.v * cos(obj.theta);
            dy = obj.v * sin(obj.theta);
            dtheta = obj.v * tan(deltat) / obj.l;
            obj.x = obj.x + dx * obj.dt;
            obj.y = obj.y + dy * obj.dt;
            obj.theta = obj.theta + dtheta * obj.dt;
        end
        
        function plot_duration(obj)
            scatter(obj.x, obj.y, 'r');
            axis([0, 18, -3, 3]);
            if is_ipython
                clear_output('wait');
                display(gcf());
            end
        end
    end
end

% set reference trajectory
refer_path = zeros(1000, 2);
refer_path(:, 1) = linspace(0, 1000, 1000)';
% refer_path(:, 2) = 5 * sin(refer_path(:, 1) / 5.0); % generating sin reference trajectory
refer_tree = KDTree(refer_path); % reference trajectory
plot(refer_path(:, 1), refer_path(:, 2), '-.b', 'LineWidth', 5.0);

% Initial: pos_x is 0, pos_y is 1.0 m, heading is 0 m,
% wheelbase is 2.0 m, speed is 2.0 m/s, decision period is 0.1s.
ugv = UGV_model(0, 1.0, 0, 2.0, 2.0, 0.1);
pind = 1;
ind = 1;
for i = 1:1000
    robot_state = zeros(2, 1);
    robot_state(1) = ugv.x;
    robot_state(2) = ugv.y;
    [~, ind] = refer_tree.query(robot_state');
    if ind < pind
        ind = pind;
    else
        pind = ind;
    end

    dist = norm(robot_state' - refer_path(ind, :));
    dx = refer_path(ind, 1) - robot_state(1);
    dy = refer_path(ind, 2) - robot_state(2);
    alpha = atan2(dy, dx);
    e = sign(sin(alpha - ugv.theta)) * dist; % bang-bang controller
    delta = sign(e) * pi / 6.0;
    ugv.update(2.0, delta);
    ugv.plot_duration();
    pause(0.001);
end
