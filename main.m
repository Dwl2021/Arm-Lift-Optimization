clear;close all;clc;
figure('Position', [200, 200, 800, 400]);
axis([0 120 0 60]);
plot_quadrotor(8,8, pi/6,5);
plot_object();



function [] = plot_object()
    x = 80;
    y = 0;
    width = 10;
    height = 25;
    hold on; % Hold the plot to draw all parts
    color = 'red';
    fill([x, x + width, x + width, x], [y, y, y + height, y + height], color); 
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
