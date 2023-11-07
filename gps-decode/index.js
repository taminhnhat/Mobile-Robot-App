const { SerialPort } = require('serialport')
const GPS = require('gps')

const port = new SerialPort({
    path: '/dev/ch341',
    baudRate: 9600
    // parser: new SerialPort.parsers.Readline({
    //     delimiter: '\r\n'
    // })
});

const gps = new GPS;

gps.on('data', data => {
    console.log(data, gps.state);
})

port.on('data', data => {
    gps.updatePartial(data);
})