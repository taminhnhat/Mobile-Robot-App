clear device;
global position;
device = serialport("COM3", 115200);
configureTerminator(device, "CR");
configureCallback(device, "terminator", @readSerialData);
configureCallback(device,"",@myErrFcn)
writeline(device, "accepted");
writeline(device, "accepted");
writeline(device, "accepted");
writeline(device, "accepted");

function readSerialData(device, evt)
    data = readline(device)
    % configureTerminator(device, "CR")
    writeline(device, "accepted");

end
