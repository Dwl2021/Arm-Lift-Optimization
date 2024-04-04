clear;close all;clc;

%% init
addpath('./minimumSnap/')
addpath('./minimum_effort')
mapsize = [200,100]; % x,y
map = ones(mapsize(1), mapsize(2));

 
% add obstacle and dilate
map = add_obstacle(map);
map_dilation = mapDilation(map, 7);
map_dilation = add_boundary(map_dilation);

% start and goal
start = [15, 15];
goal = [180, 48];

% plot map
x_obstacle = [];
y_obstacle = [];
for i = 1:mapsize(1)
    for j = 1:mapsize(2)
        if map(i,j) == 2
            x_obstacle = [x_obstacle, i];
            y_obstacle = [y_obstacle, j];
        end
    end
end



%% path searching
[path, goal_reached, cost, EXPAND] = jps(map_dilation, start, goal);
path = flip(path);
if ~goal_reached
    error('Goal not reached!'); 
end


%% Minimum Snap Trajectory
[coff_x,coff_y,ts] = QPSolver(path, 40, 10, 0.1, 7);
X_n = [];
Y_n = [];
k = 1;
n_seg = size(path,1)-1;
tstep = 0.01;
n_order = 7;
for i=0:n_seg-1
    Pxi = coff_x((n_order+1)*i+1:(n_order+1)*i+n_order+1); % note (n_order+1) jump to another segment!
    Pyi = coff_y((n_order+1)*i+1:(n_order+1)*i+n_order+1);
    for t=0:tstep:ts(i+1)
        X_n(k)  = polyval(flip(Pxi),t);
        Y_n(k)  = polyval(flip(Pyi),t);
        k = k+1;
    end
end

%% manipulator working space
init = 7;
working_space = [];
for i = 1:size(X_n,2)
    if mod(i,20)~=0
        continue;
    end
    x = X_n(i);
    y = Y_n(i);
    for j=10:-1:3
        if y-j <=0
            continue;
        end
        if map(round(x),round(y-j)) == 1
            working_space = [working_space, j];
            break;
        end
    end
end

n_order = 7;
coff = minimum_effort(working_space, n_order);
len_arms = zeros(1, size(X_n,2));
n_segs = length(working_space) - 1;
for i = 1:size(X_n,2)
    segs = floor(i/20);
    if segs >= n_segs
        segs = n_segs-1;
    end
    tstep = 1 / 20;
    P = coff((n_order+1)*segs+1:(n_order+1)*segs+(n_order+1));
    len_arms(i) = polyval(flip(P),tstep * mod(i,20));
end



% 创建一个 figure 并设置位置和坐标轴
figure('Position', [200, 200, 800, 800 * mapsize(2) / mapsize(1)]);
axis([0 mapsize(1) 0 mapsize(2)]);
hold on;

% 开始动画
for i = 1:size(X_n, 2)
    clf;
    
    % 更新绘图数据，不清空画布
    plot(X_n, Y_n, 'Color', '#FFA500', 'LineWidth', 5);
    hold on;
    scatter(x_obstacle, y_obstacle, 10, 'filled', 'MarkerFaceColor', 'black');
    hold on;
    scatter(path(:,1), path(:,2), 'filled', 'MarkerFaceColor', '#1E90FF', 'SizeData', 200, 'Marker', '^');
    hold on;
    plot_quadrotor(X_n(i), Y_n(i), 0 , len_arms(i));
    % 左上角加上arms长度
    text(8, 80, ['arms length: ', num2str(len_arms(i))], 'FontSize', 20);
    axis([0 mapsize(1) 0 mapsize(2)]);
    drawnow;
    pause(0.01);
end




function [map] = add_obstacle(map)
    x = size(map,1) *0.8;
    y = 1;
    width = size(map,1) * 0.1;
    height = size(map,2) * 0.4;
    for i = x:x+width
        for j = y:y+height
            map(i,j) = 2;
        end
    end

    x = size(map,1) *0.5;
    y = 1;
    width = size(map,1) * 0.1;
    height = size(map,2) * 0.7;
    for i = x:x+width
        for j = y:y+height
            map(i,j) = 2;
        end
    end

    x = size(map,1) *0.5;
    y = size(map,2);
    width = size(map,1) * 0.1;
    height = size(map,2) * 0.1;
    for i = x:x+width
        for j = y-height:y
            map(i,j) = 2;
        end
    end
end

function [map] = add_boundary(map)
    map(1,:) = 2;
    map(end,:) = 2;
    map(:,1) = 2;
    map(:,end) = 2;
end


function [] = plot_quadrotor(x, y, theta, armlength)
    
%     if nargin < 4 || theta < - pi/6 || theta > pi/6 || armlength < 1 || armlength > 11
%         error('[plot_quadrotor]Error Input!');
%     end
    wind = 4;
    l = 8;
    h = 2;
    body = [x - l/2, y; x + l/2, y];
    arm1 = [x - l/2, y; x - l/2, y + h];
    arm2 = [x + l/2, y; x + l/2, y + h];
    arm3 = [x, y - 2; x , y];
    arm4 = [x, y - 2 ; x , y - armlength - 2];
    prop1 = [x - l/2 - wind/2, y + h; x - l/2 + wind/2, y + h];
    prop2 = [x + l/2 - wind/2, y + h; x + l/2 + wind/2, y + h];
    hold on; % Hold the plot to draw all parts
    color = 'black';
    lineWidth = 2;
    plot(body(:,1), body(:,2), 'Color', color, 'LineWidth', lineWidth);
    plot(arm1(:,1), arm1(:,2), 'Color', color, 'LineWidth', lineWidth);
    plot(arm2(:,1), arm2(:,2), 'Color', color, 'LineWidth', lineWidth);
    plot(arm3(:,1), arm3(:,2), 'Color', color, 'LineWidth', lineWidth);
    plot(arm4(:,1), arm4(:,2), 'Color', 'black', 'LineWidth', 1); % #1E90FF
    plot(arm4(2,1), arm4(2,2), 'o', 'Color', '#7FFF00', 'MarkerSize', 6, 'MarkerFaceColor', '#7FFF00');
    plot(prop1(:,1), prop1(:,2), 'Color', color, 'LineWidth', lineWidth);
    plot(prop2(:,1), prop2(:,2), 'Color', color, 'LineWidth', lineWidth);
end

function dilatedMap = mapDilation(Map, dilationRadius)
    binaryMap = Map == 2;
    se = strel('disk', dilationRadius);
    dilatedMap = imdilate(binaryMap, se);
    dilatedMap = double(dilatedMap);
    dilatedMap(dilatedMap == 1) = 2;
end
