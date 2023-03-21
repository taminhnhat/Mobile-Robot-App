clear device;
global position;

clear server;
address = "192.168.1.42";
port = 9600;
server = tcpserver(address,port,"ConnectionChangedFcn",@connectionFcn);
% configureTerminator(server, "CR/LF");
configureCallback(server,"terminator",@callbackFcn);

warehouseModel();

position.x = 13;
position.y = 1;
global p;
p = robotShow(position);
plot(position.x, position.y, 'r*');

% destination = pathParse('RUN:1300:3000:5000/');
% position = simulate(position, destination);
% res = sprintf("POS:%.1d:%.1d\n", [round(position.x * 100), round(position.y * 100)]);
% write(device, res, "string");

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
    p = rectangle('Position',[pos.x-rWidth/2 pos.y-rLength/2 rWidth rLength],'FaceColor',[.92 .63 .11],'EdgeColor',[0 0 0]);
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

function connectionFcn(src,~)
    if src.Connected
        disp("Client connected");
    else
        disp("Client has disconnected.");
    end
end

function callbackFcn(src,evt)
    disp(src);
    global position;
    message = readline(src);
    mesParse = regexp(message, '\w*','match');
    switch mesParse(1)
        case 'RUN'
            fprintf("X%sY%sV%s\n",[mesParse(2),mesParse(3),mesParse(4)]);
            des.x = str2num(cell2mat(mesParse(2)));
            des.y = str2num(cell2mat(mesParse(3)));
            des.v = str2num(cell2mat(mesParse(4)));
            position = simulate(position,des);
            res = sprintf("POS:%d:%d", [round(position.x), round(position.y)]);
            src.writeline(res);
        case 'STATUS'
            disp('status');
        otherwise
            disp('not a valid message');
    end
end
