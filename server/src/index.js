const app = require('./app')
const http = require("http")
const path = require('path')
const server = http.createServer(app)
const port = Number(process.env.HTTP_PORT) || 3003
const io = require('socket.io')(server)

io.on('connection', onConnection);
function onConnection(socket) {
    const msg = `New bot ${socket.conn.id}|${socket.conn.remoteAddress} connected`

    // socket.on('robot:connect', robotCtrl.onConnect)
    socket.on('ros:topic', d => {
        socket.broadcast.emit('ros:topic', d)// send to all connected clients except the sender
    })
    socket.on('ros:monitor', d => {
        socket.broadcast.emit('ros:monitor', d)// send to all connected clients except the sender
    })
    socket.on('gamepad', d => {
        // console.log(d.topic, d.data)
        socket.broadcast.emit('gamepad', d)// send to all connected clients except the sender
    })
    socket.on('robot:camera:start', () => {
        socket.broadcast.emit('robot:camera:start', {})
    })
    socket.on('robot:camera:stop', () => {
        socket.broadcast.emit('robot:camera:stop', {})
    })
    // socket.on('robot:message', robotCtrl.onMessage)
    setInterval(() => {
        // socket.emit('ros:topic', 'control')
    }, 1000)
}

app.get('/', (req, res) => {
    res.sendFile(__dirname + '/templates/controller.html');
})
app.get('/test', (req, res) => {
    res.sendFile(__dirname + '/templates/gamepadTest.html');
})
app.get('/video', (req, res) => {
    res.sendFile(__dirname + '/templates/video.html');
})
app.get('/src', (req, res) => {
    res.sendFile(path.resolve(__dirname, '..') + '/public/' + req.query.path);
})

server.listen(port, () => {
    console.log(`Robot server running at http://localhost:${port}/`);
});