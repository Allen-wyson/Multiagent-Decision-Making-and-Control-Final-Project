%% run_random_steps.m
% This script visualizes two cars on a 3×50 grid racetrack.
% At each step, it picks a random “snake” move for each car,
% draws the planned paths as thinner dashed lines, marks each
% predicted cell with a small 'x', waits for you to press Enter,
% then moves the cars to the next positions.

clc; clear; close all;

%% Parameters
numLanes    = 3;    % number of lanes
trackLength = 50;   % track length
maxD        = 3;    % max substeps per large step
maxSteps    = 20;   % maximum simulation steps

%% Initialize vehicle states
car1.lane = 2; car1.pos = 1;
car2.lane = 3; car2.pos = 1;

%% Draw racetrack
ax = create_racetrack(numLanes, trackLength);
hold(ax,'on');

% Plot initial car positions
h1 = plot(ax, car1.pos-0.5, car1.lane-0.5, 'bo', ...
          'MarkerSize',10, 'LineWidth',2);
h2 = plot(ax, car2.pos-0.5, car2.lane-0.5, 'rs', ...
          'MarkerSize',10, 'LineWidth',2);

%% Main loop: random moves --> to be replaced with SE-IBR algorithm
for step = 1:maxSteps
    if car1.pos >= trackLength || car2.pos >= trackLength
        break;  % stop if any car reached finish
    end

    %--- enumerate possible actions for each car ---
    actions1 = enumerateActions(car1.lane, car1.pos, maxD, numLanes, trackLength);
    actions2 = enumerateActions(car2.lane, car2.pos, maxD, numLanes, trackLength);

    %--- pick one random action from each set ---
    idx1 = randi(numel(actions1));
    idx2 = randi(numel(actions2));
    plan1 = actions1{idx1};   % k×2 array of [lane,pos]

    
    plan2 = actions2{idx2};

    %--- delete previous dashed paths and x‐markers if any ---
    if exist('h1_path','var') && isvalid(h1_path), delete(h1_path); end
    if exist('h2_path','var') && isvalid(h2_path), delete(h2_path); end
    if exist('h1_x','var')    && isvalid(h1_x),    delete(h1_x);    end
    if exist('h2_x','var')    && isvalid(h2_x),    delete(h2_x);    end

    %--- plot planned paths as thinner dashed lines ---
    x1 = plan1(:,2) - 0.5;
    y1 = plan1(:,1) - 0.5;
    h1_path = plot(ax, x1, y1, 'b--', 'LineWidth',1);

    x2 = plan2(:,2) - 0.5;
    y2 = plan2(:,1) - 0.5;
    h2_path = plot(ax, x2, y2, 'r--', 'LineWidth',1);

    %--- mark each predicted cell with a small 'x' ---
    h1_x = plot(ax, x1, y1, 'bx', 'MarkerSize',8, 'LineWidth',1);
    h2_x = plot(ax, x2, y2, 'rx', 'MarkerSize',8, 'LineWidth',1);

    %--- wait for user to press Enter ---
    input('Press Enter to execute the move','s');

    %--- execute the first (and only) large step ---
    car1.lane = plan1(end,1);
    car1.pos  = plan1(end,2);
    car2.lane = plan2(end,1);
    car2.pos  = plan2(end,2);

    %--- update car markers ---
    set(h1, 'XData', car1.pos-0.5, 'YData', car1.lane-0.5);
    set(h2, 'XData', car2.pos-0.5, 'YData', car2.lane-0.5);

    drawnow;
end

%% Display result
if car1.pos >= trackLength
    disp('Car 1 reached the finish!');
elseif car2.pos >= trackLength
    disp('Car 2 reached the finish!');
else
    disp('Simulation ended (max steps reached).');
end
