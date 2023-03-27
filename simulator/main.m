global position;
global R01;
global R02;
global R03;
global robot;

clear server;
address = "192.168.1.42";
port = 8080;
server = tcpserver(address, port, "ConnectionChangedFcn", @connectionFcn);
% configureTerminator(server, "CR/LF");
configureCallback(server, "terminator", @callbackFcn);

warehouseModel();

position.x = 13;
position.y = 1;
robot(1) = Robot("R-01", [235 160 28], position);
position.x = 15;
position.y = 1;
robot(2) = Robot("R-02", [39 186 245], position);
position.x = 17;
position.y = 1;
robot(3) = Robot("R-03", [64 152 52], position);
global p;
robot(1).show();
robot(2).show();
robot(3).show();

function connectionFcn(src, ~)
    global robot;

    if src.Connected
        disp("Client connected");
        res = robot(1).getPos();
        mess = sprintf("R01:POS:%d:%d", [round(res.x), round(res.y)]);
        src.writeline(mess);
        res = robot(2).getPos();
        mess = sprintf("R02:POS:%d:%d", [round(res.x), round(res.y)]);
        src.writeline(mess);
        res = robot(3).getPos();
        mess = sprintf("R03:POS:%d:%d", [round(res.x), round(res.y)]);
        src.writeline(mess);
    else
        disp("Client has disconnected.");
    end

end

function callbackFcn(src, evt)
    global position;
    global robot;
    message = readline(src);
    mesParse = regexp(message, '\w*', 'match');

    switch mesParse(2)
        case 'RUN'
            fprintf("X%sY%sV%s\n", [mesParse(3), mesParse(4), mesParse(5)]);
            des.x = str2num(cell2mat(mesParse(3)));
            des.y = str2num(cell2mat(mesParse(4)));
            des.v = str2num(cell2mat(mesParse(5)));
            res = '';

            switch mesParse(1)
                case 'R01'
                    res = robot(1).simulate(des);
                    mess = sprintf("R01:POS:%d:%d", [round(res.x), round(res.y)]);
                case 'R02'
                    res = robot(2).simulate(des);
                    mess = sprintf("R02:POS:%d:%d", [round(res.x), round(res.y)]);
                case 'R03'
                    res = robot(3).simulate(des);
                    mess = sprintf("R03:POS:%d:%d", [round(res.x), round(res.y)]);
                otherwise
                    return;
            end

            src.writeline(mess);
        case 'STATUS'
            disp('status');
        otherwise
            disp('not a valid message');
    end

end
