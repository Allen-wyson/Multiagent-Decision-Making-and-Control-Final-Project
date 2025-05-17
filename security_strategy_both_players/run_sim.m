clc; clear; close all;

numLanes = 3;
trackLength = 50;
maxD = 3;
maxSteps = 20;

car1.lane = 2; car1.pos = 1;
car2.lane = 3; car2.pos = 2;

ax = create_racetrack(numLanes, trackLength);
hold(ax,'on');

h1 = plot(ax, car1.pos-0.5, car1.lane-0.5, 'bo', 'MarkerSize',10, 'LineWidth',2);
h2 = plot(ax, car2.pos-0.5, car2.lane-0.5, 'rs', 'MarkerSize',10, 'LineWidth',2);

for step = 1:maxSteps
    if car1.pos >= trackLength || car2.pos >= trackLength
        break;
    end

    actions1 = enumerateActions(car1.lane, car1.pos, maxD, numLanes, trackLength);
    actions2 = enumerateActions(car2.lane, car2.pos, maxD, numLanes, trackLength);

    [plan1, plan2] = securityStrategyPlanner(actions1, actions2, trackLength);

    if exist('h1_path','var') && isvalid(h1_path), delete(h1_path); end
    if exist('h2_path','var') && isvalid(h2_path), delete(h2_path); end
    if exist('h1_x','var') && isvalid(h1_x), delete(h1_x); end
    if exist('h2_x','var') && isvalid(h2_x), delete(h2_x); end

    x1 = plan1(:,2) - 0.5; y1 = plan1(:,1) - 0.5;
    x2 = plan2(:,2) - 0.5; y2 = plan2(:,1) - 0.5;
    h1_path = plot(ax, x1, y1, 'b--', 'LineWidth',1);
    h2_path = plot(ax, x2, y2, 'r--', 'LineWidth',1);
    h1_x = plot(ax, x1, y1, 'bx', 'MarkerSize',8, 'LineWidth',1);
    h2_x = plot(ax, x2, y2, 'rx', 'MarkerSize',8, 'LineWidth',1);

    input('Press Enter to execute the move','s');

    car1.lane = plan1(end,1); car1.pos = plan1(end,2);
    car2.lane = plan2(end,1); car2.pos = plan2(end,2);

    set(h1, 'XData', car1.pos-0.5, 'YData', car1.lane-0.5);
    set(h2, 'XData', car2.pos-0.5, 'YData', car2.lane-0.5);
    drawnow;
end

if car1.pos >= trackLength
    disp('Car 1 reached the finish!');
elseif car2.pos >= trackLength
    disp('Car 2 reached the finish!');
else
    disp('Simulation ended (max steps reached).');
end
