function ax = create_racetrack(numLanes, trackLength)
    figure;
    ax = gca;
    hold on;
    axis equal;
    axis([0 trackLength 0 numLanes]);
    xticks(0:trackLength);
    yticks(0:numLanes);
    grid on;
    title('Coupled Minimax Race Simulation');
end
