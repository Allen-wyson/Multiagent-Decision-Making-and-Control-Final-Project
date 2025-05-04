function actions = enumerateActions(lane0, pos0, maxD)
% enumerateActions lists all possible “snake” moves from (lane0,pos0)
%   maxD: maximum sub-steps (1~3)

    if nargin < 3
        maxD = 3;
    end

    actions = {};  % cell array to hold each action path
    path = [];
    dfs(lane0, pos0, maxD, path);
    
    function dfs(curLane, curPos, stepsLeft, path)
        if ~isempty(path)
            actions{end+1} = path;  %#ok<AGROW>
        end
        if stepsLeft == 0
            return;
        end
        for dl = -1:1
            nextLane = curLane + dl;
            if nextLane >=1 && nextLane <=3
                nextPos = curPos + 1;
                newPath = [path; nextLane, nextPos];
                dfs(nextLane, nextPos, stepsLeft-1, newPath);
            end
        end
    end
end
