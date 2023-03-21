device = serialport("COM3", 115200);
configureTerminator(device, "CR");
configureCallback(device, "terminator", @readSerialData);
function readSerialData(src, evt)
    data = readline(src);
    src.UserData = data;
end