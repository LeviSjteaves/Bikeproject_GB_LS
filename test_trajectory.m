function  Create_trajectory()
% Create a GUI with buttons to plot different shapes, set their initial positions,
% and save their data
clc;
close all;
clear all;

% Set the default number of datapoints and initial positions
default_n = 100;
default_scale = 1;
default_x = 0;
default_y = 0;

        filename = 'trajectory.txt';
       %save_data(filename, x, y)

% Create the main window and set its size
figure('Name','GUI','Position', [50, 100, 500, 500]);

% Create an edit box to enter number of points
edit_n = uicontrol('Style', 'edit', 'String', num2str(default_n), ...
                   'Position', [350, 450, 50, 30]);

% Create an edit box to enter line settings
startline_x = uicontrol('Style', 'edit', 'String', num2str(default_x), ...
                   'Position', [100, 400, 50, 30]);
startline_y = uicontrol('Style', 'edit', 'String', num2str(default_y), ...
                   'Position', [150, 400, 50, 30]);
endline_x = uicontrol('Style', 'edit', 'String', num2str(default_x), ...
                   'Position', [200, 400, 50, 30]);
endline_y = uicontrol('Style', 'edit', 'String', num2str(default_y), ...
                   'Position', [250, 400, 50, 30]);
% Create an edit box to enter curve settings

% Create an edit box to enter circle settings
startcir_x = uicontrol('Style', 'edit', 'String', num2str(default_x), ...
                   'Position', [100, 300, 50, 30]);
startcir_y = uicontrol('Style', 'edit', 'String', num2str(default_y), ...
                   'Position', [150, 300, 50, 30]);
edit_radiuscir = uicontrol('Style', 'edit', 'String', num2str(default_scale), ...
                   'Position', [200, 300, 50, 30]);

% Create an edit box to enter infinite settings
startsin_x = uicontrol('Style', 'edit', 'String', num2str(default_x), ...
                   'Position', [100, 250, 50, 30]);
startsin_y = uicontrol('Style', 'edit', 'String', num2str(default_y), ...
                   'Position', [150, 250, 50, 30]);
sin_amp = uicontrol('Style', 'edit', 'String', num2str(default_scale), ...
                   'Position', [200, 250, 50, 30]);
sin_freq = uicontrol('Style', 'edit', 'String', num2str(default_scale), ...
                   'Position', [250, 250, 50, 30]);
sin_rep = uicontrol('Style', 'edit', 'String', num2str(default_scale), ...
                   'Position', [300, 250, 50, 30]);


% Create an edit box to enter infinite settings
edit_radiusinf = uicontrol('Style', 'edit', 'String', num2str(default_scale), ...
                   'Position', [100, 200, 50, 30]);
startinf_x = uicontrol('Style', 'edit', 'String', num2str(default_x), ...
                   'Position', [150, 200, 50, 30]);
startinf_y = uicontrol('Style', 'edit', 'String', num2str(default_y), ...
                   'Position', [200, 200, 50, 30]);


% Create buttons to plot each shape
uicontrol('Style', 'pushbutton', 'String', 'Line', ...
          'Position', [50, 400, 50, 50], 'Callback', @plot_line);
uicontrol('Style', 'pushbutton', 'String', 'Curve', ...
          'Position', [50, 350, 50, 50], 'Callback', @plot_curve);
uicontrol('Style', 'pushbutton', 'String', 'Circle', ...
          'Position', [50, 300, 50, 50], 'Callback', @plot_circle);
uicontrol('Style', 'pushbutton', 'String', 'Sinus', ...
          'Position', [50, 250, 50, 50], 'Callback', @plot_sinus);
uicontrol('Style', 'pushbutton', 'String', 'infinite', ...
          'Position', [50, 200, 50, 50], 'Callback', @plot_infinite);
uicontrol('Style', 'pushbutton', 'String', 'Save', ...
          'Position', [50, 150, 50, 50], 'Callback', @save_data);

% Initialize the data matrix to an empty array
data = [];
figure('Name','Example of segment','Position', [550, 100, 500, 500])
% Define the functions to plot each shape
       function [x, y] = plot_line(~, ~)
        n = str2double(get(edit_n, 'String'));
        x_init = str2double(get(startline_x, 'String'));
        y_init = str2double(get(startline_y, 'String'));
        x_end = str2double(get(endline_x, 'String'));
        y_end = str2double(get(endline_y, 'String'));
        x = linspace(x_init, x_end, n) ;
        y = linspace(y_init, y_end, n);
        figure(2)
        plot(x, y);
        hold on;
        plot(x(1), y(1), 'o');
        hold off
        title('Line');
        data = [x; y];
       end
    
    function [x, y] = plot_circle(~, ~)
        radius = str2double(get(edit_radiuscir, 'String'));
        n = str2double(get(edit_n, 'String'));
        x_init = str2double(get(startcir_x, 'String'));
        y_init = str2double(get(startcir_y, 'String'));
        theta = linspace(0, 2*pi, n);
        x = radius*cos(theta) + x_init - radius;
        y = radius*sin(theta) +  y_init;
        figure(2)
        plot(x, y);
        hold on;
        plot(x(1), y(1), 'o');
        hold off;
        title('Circle');
        data = [x; y];
    end

    function [x, y] = plot_infinite(~, ~)
        radius = str2double(get(edit_radiusinf, 'String'));
        n = str2double(get(edit_n, 'String'));
        x_init = str2double(get(startinf_x, 'String'));
        y_init = str2double(get(startinf_y, 'String'));
        theta = linspace(0, 2*pi, n);
        x = radius*cos(theta) + x_init - radius;
        y = radius*sin(2*theta) / 2 +  y_init;
        figure(2)
        plot(x, y);
        hold on;
        plot(x(1), y(1), 'o');
        hold off;
        title('Infinite');
        data = [x; y];
    end

    function [x, y] = plot_sinus(~, ~)
        n = str2double(get(edit_n, 'String'));
        rep = str2double(get(sin_rep, 'String'));
        amplitude = str2double(get(sin_amp, 'String'));
        freq = str2double(get(sin_freq, 'String'));
        x_init = str2double(get(startsin_x, 'String'));
        y_init = str2double(get(startsin_y, 'String'));
        x = linspace(0, (2*pi*rep)/freq, n)+x_init;
        y = amplitude*sin(freq*x)+y_init;
        figure(2)
        plot(x, y);
        hold on;
        plot(x(1), y(1), 'o');
        hold off
        title('Sinus Wave');
        data = [x; y];
       end

    function [x, y] = plot_curve(~, ~)
        n = str2double(get(edit_n, 'String'));
        x = linspace(-1, 1, n);
        y = x.^3 - x.^2 + x;
        figure(2)
        plot(x, y);
        hold on;
        plot(x(1), y(1), 'o');
        hold off
        title('Curve');
        data = [x; y];
       end

%     function save_button(~, ~)
%         filename = 'trajectory.txt';
%         save_data(filename, x, y)
%     end

% Define a function to save the x and y data to a file
    function save_data(~, ~)
        dlmwrite(filename, data', 'delimiter', '\t');
        figure(3)
        hold on
        plot(data(1,:),data(2,:))
        hold on
    end

    end
