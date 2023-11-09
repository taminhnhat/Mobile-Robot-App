require('dotenv').config({ path: '../.env' })
const serverPort = Number(process.env.SERVER_PORT) || 3001
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);

var Sylvester = require('sylvester');
var Kalman = require('kalman').KF;

const { SerialPort } = require('serialport');

const port = process.env.GPS_PORT
const serialPort = new SerialPort({
    path: port,
    baudRate: 9600
});


var GPS = require('gps');
var gps = new GPS;

// Simple Kalman Filter set up
var A = Sylvester.Matrix.I(2);
var B = Sylvester.Matrix.Zero(2, 2);
var H = Sylvester.Matrix.I(2);
var C = Sylvester.Matrix.I(2);
var Q = Sylvester.Matrix.I(2).multiply(1e-11);
var R = Sylvester.Matrix.I(2).multiply(0.00001);

// Measure
var u = $V([0, 0]);

var filter = new Kalman($V([0, 0]), $M([[1, 0], [0, 1]]));

gps.on('data', function (data) {
    console.log('<<<', data.raw)

    try {
        if (data.lat && data.lon) {

            filter.update({
                A: A,
                B: B,
                C: C,
                H: H,
                R: R,
                Q: Q,
                u: u,
                y: $V([data.lat, data.lon])
            });

            gps.state.position = {
                cov: filter.P.elements,
                pos: filter.x.elements
            };
        }
        // console.log('gps', gps.state)
        io.emit('position', gps.state);
    }
    catch (err) {
        console.log(err)
    }
});

app.get('/', function (req, res) {
    res.sendFile(__dirname + '/maps.html');
});

http.listen(serverPort, function () {
    console.log(`listening on port ${serverPort}`);
});

serialPort.on('data', function (data) {
    // console.log(String(data).trim())
    gps.updatePartial(data);
});
