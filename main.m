clear;close all;clc;

mapsize = [200,100] % x,y
map = ones(mapsize(1), mapsize(2));
figure('Position', [200, 200, 800, 800 * mapsize(2) / mapsize(1)]);
axis([0 mapsize(1) 0 mapsize(2)]);
hold on;
map = add_obstacle(map);
map_dilation = mapDilation(map, 7);
map_dilation = add_boundary(map_dilation);

start = [10, 10];
goal = [150, 48];

[path, goal_reached, cost, EXPAND] = jps(map_dilation, start, goal);

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
scatter(x_obstacle, y_obstacle, 10, 'filled', 'MarkerFaceColor', 'black');
hold on;

if goal_reached
    plot(path(:,1), path(:,2), 'Color', 'red', 'LineWidth', 2);
end


plot_quadrotor(start(1),start(2)+2, pi/6,5);



function [map] = add_obstacle(map)
    x = size(map,1) *0.7;
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
    if nargin < 4 || theta < - pi/6 || theta > pi/6 || armlength < 2 || armlength > 6
        error('[plot_quadrotor]Error Input!');
    end
    wind = 4;
    l = 8;
    h = 2;
    body = [x - l/2, y; x + l/2, y];
    arm1 = [x - l/2, y; x - l/2, y + h];
    arm2 = [x + l/2, y; x + l/2, y + h];
    arm3 = [x, y - 2; x , y];
    arm4 = [x, y - 2 ; x + armlength * sin(theta), y - armlength * cos(theta) - 2];
    arm4
    prop1 = [x - l/2 - wind/2, y + h; x - l/2 + wind/2, y + h];
    prop2 = [x + l/2 - wind/2, y + h; x + l/2 + wind/2, y + h];
    hold on; % Hold the plot to draw all parts
    color = 'black';
    lineWidth = 2;
    plot(body(:,1), body(:,2), 'Color', color, 'LineWidth', lineWidth);
    plot(arm1(:,1), arm1(:,2), 'Color', color, 'LineWidth', lineWidth);
    plot(arm2(:,1), arm2(:,2), 'Color', color, 'LineWidth', lineWidth);
    plot(arm3(:,1), arm3(:,2), 'Color', color, 'LineWidth', lineWidth);
    plot(arm4(:,1), arm4(:,2), 'Color', '#1E90FF', 'LineWidth', 4);
    plot(arm4(2,1), arm4(2,2), 'o', 'Color', '#7FFF00', 'MarkerSize', 10, 'MarkerFaceColor', '#7FFF00');
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
