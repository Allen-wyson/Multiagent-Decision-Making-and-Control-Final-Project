function actions = enumerateActions(lane0, pos0, maxD, numLanes, trackLength)
    narginchk(3,5); 
    if nargin < 4, numLanes = 3; end
    if nargin < 5, trackLength = 50; end
    assert(ismember(lane0,1:numLanes));
    assert(pos0 >= 1 && pos0 < trackLength && pos0==floor(pos0));
    assert(maxD >= 1 && maxD == floor(maxD));

    actions = {};
    dfs(lane0, pos0, maxD, []);

    function dfs(curLane, curPos, stepsLeft, path)
        if ~isempty(path)
            actions{end+1} = path;
        end
        if stepsLeft == 0
            return;
        end
        for dl = -1:1
            nextLane = curLane + dl;
            nextPos = curPos + 1;
            if nextLane>=1 && nextLane<=numLanes && nextPos<=trackLength
                newPath = [path; nextLane, nextPos];
                dfs(nextLane, nextPos, stepsLeft-1, newPath);
            end
        end
    end
end
