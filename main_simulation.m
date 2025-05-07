clc; clear; close all;

% Initialize vehicle state
car1.lane = 2;    % starting lane (1~3)
car1.pos  = 1;    % starting position

car2.lane = 3;    
car2.pos  = 1;

create_racetrack;
% after drawing the track:
hold on;

% plot car1 as a blue circle
h1 = plot(car1.pos-0.5, car1.lane-0.5, 'bo', 'MarkerSize',12, 'LineWidth',2);

% plot car2 as a magenta square
h2 = plot(car2.pos-0.5, car2.lane-0.5, 'ms', 'MarkerSize',12, 'LineWidth',2);

drawnow;


car1Plan = enumerateActions(car1.lane, car1.pos, 3);
car2Plan = enumerateActions(car2.lane, car2.pos, 3);

% Example: car1 does first action of its plan
action1 = car1Plan{1};   % a k×2 matrix
% take only the last substep
nextState1 = action1(end, :);  % [lane, pos]
car1.lane = nextState1(1);
car1.pos  = nextState1(2);

% same for car2
action2 = car2Plan{1};
nextState2 = action2(end, :);
car2.lane = nextState2(1);
car2.pos  = nextState2(2);

% update frame
set(h1, 'XData', car1.pos-0.5, 'YData', car1.lane-0.5);
set(h2, 'XData', car2.pos-0.5, 'YData', car2.lane-0.5);
drawnow;
