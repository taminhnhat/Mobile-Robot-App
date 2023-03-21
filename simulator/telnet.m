clear server;
address = "192.168.1.42";
port = 9600;
server = tcpserver(address,port);
% configureTerminator(server, "CR/LF");
configureCallback(server,"terminator",@callbackFcn);
function callbackFcn(src,~)
    message = readline(src);
    disp(message)
end