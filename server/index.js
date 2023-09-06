const app = require('./app')
const http = require("http")
const server = http.createServer(app)
const port = Number(process.env.HTTP_PORT) || 3003
const io = require('socket.io')(server)

io.on('connection', onConnection);
function onConnection(socket) {
    const msg = `New bot ${socket.conn.id}|${socket.conn.remoteAddress} connected`

    // socket.on('robot:connect', robotCtrl.onConnect)
    socket.on('ros:topic', d => {
        // console.log(d.topic, d.data)
        socket.broadcast.emit('ros:topic', d)// send to all connected clients except the sender
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
app.get('/src', (req, res) => {
    res.sendFile(__dirname + '/src/' + req.query.path);
})

server.listen(port, () => {
    console.log(`Robot server running at http://localhost:${port}/`);
});