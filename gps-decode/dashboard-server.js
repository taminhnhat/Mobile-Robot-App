
const app = require('express')();
const http = require('http').Server(app);
const io = require('socket.io')(http);
require('dotenv').config({ path: '.env' })

const { SerialPort } = require('serialport');

const port = process.env.GPS_PORT
const serialPort = new SerialPort({
    path: port,
    baudRate: 9600
});

app.get('/', function (req, res) {
    res.sendFile(__dirname + '/dashboard.html');
});

var GPS = require('gps');
var gps = new GPS;
gps.state.bearing = 0;
var prev = { lat: null, lon: null };

http.listen(3000, function () {

    console.log('listening on *:3000');

    gps.on('data', function () {
        if (prev.lat !== null && prev.lon !== null) {
            gps.state.bearing = GPS.Heading(prev.lat, prev.lon, gps.state.lat, gps.state.lon);
        }
        io.emit('state', gps.state);
        prev.lat = gps.state.lat;
        prev.lon = gps.state.lon;
        ;
    });

    serialPort.on('data', function (data) {
        gps.updatePartial(data);
    });
});
