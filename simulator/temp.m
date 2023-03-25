function res = pathParse(str)
    rexgexpResult = regexp(str, '\d*', 'Match')
    % rexgexpResult = regexp(str, '([0-9]+)', 'Match') % anotherway
    % Get value from message
    res.x = str2num(cell2mat(rexgexpResult(1)));
    res.y = str2num(cell2mat(rexgexpResult(2)));
    res.v = str2num(cell2mat(rexgexpResult(3)));
    % another way
    % str2num([rexgexpResult{1, 1}])
    % str2num([rexgexpResult{1, 2}])
    % str2num([rexgexpResult{1, 3}])
end

function p = robotShow(pos)
    rWidth = 0.5;
    rLength = 0.6;
    p = rectangle('Position', [pos.x - rWidth / 2 pos.y - rLength / 2 rWidth rLength], 'FaceColor', [.92 .63 .11], 'EdgeColor', [0 0 0]);
    hold on;
end

function result = simulate(startPoint, endPoint)
    global p;
    t_samp = 0.001;
    velocity = endPoint.v;
    nx = (endPoint.x - startPoint.x) / (velocity * t_samp);
    ny = (endPoint.y - startPoint.y) / (velocity * t_samp);
    n = abs(nx);

    if (abs(ny) > abs(nx))
        n = abs(ny);
    end

    n = round(n);

    if (n == 0 || n == 1)
        delete(p);
        p = robotShow(endPoint);
        hold on;
        result.x = endPoint.x;
        result.y = endPoint.y;
    elseif (n > 1)
        dx = (endPoint.x - startPoint.x) / n;
        dy = (endPoint.y - startPoint.y) / n;
        path.x(1) = startPoint.x + dx;
        path.y(1) = startPoint.y + dy;
        pause(t_samp);

        for i = 2:n
            path.x(i) = path.x(i - 1) + dx;
            path.y(i) = path.y(i - 1) + dy;
            delete(p);
            tmp.x = path.x(i);
            tmp.y = path.y(i);
            p = robotShow(tmp);
            hold on;
            pause(t_samp);
        end

        result.x = path.x(n);
        result.y = path.y(n);
    else
        disp("error");
    end

end
