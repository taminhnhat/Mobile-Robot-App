global position;
global R01;
global R02;
global R03;

clear server;
address = "192.168.1.42";
port = 8080;
server = tcpserver(address, port, "ConnectionChangedFcn", @connectionFcn);
% configureTerminator(server, "CR/LF");
configureCallback(server, "terminator", @callbackFcn);

warehouseModel();

position.x = 13;
position.y = 1;
R01 = Robot("R-01", [235 160 28], position);
position.x = 15;
position.y = 1;
R02 = Robot("R-02", [39 186 245], position);
position.x = 17;
position.y = 1;
R03 = Robot("R-03", [64 152 52], position);
global p;
p1 = R01.show();
p2 = R02.show();
p3 = R03.show();
des.x = 19;
des.y = 1;
des.v = 100;
R01.simulate(des)
p1 = R01.show();

function connectionFcn(src, ~)

    if src.Connected
        disp("Client connected");
    else
        disp("Client has disconnected.");
    end

end

function callbackFcn(src, evt)
    global position;
    global R01;
    global p1;
    message = readline(src);
    mesParse = regexp(message, '\w*', 'match');

    switch mesParse(1)
        case 'RUN'
            fprintf("X%sY%sV%s\n", [mesParse(2), mesParse(3), mesParse(4)]);
            des.x = str2num(cell2mat(mesParse(2)));
            des.y = str2num(cell2mat(mesParse(3)));
            des.v = str2num(cell2mat(mesParse(4)));
            % position = simulate(position, des);
            delete(p1);
            R01.simulate(des);
            p1 = R01.show();
            res = sprintf("POS:%d:%d", [round(position.x), round(position.y)]);
            src.writeline(res);
        case 'STATUS'
            disp('status');
        otherwise
            disp('not a valid message');
    end

end
