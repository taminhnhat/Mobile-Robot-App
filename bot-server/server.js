const app = require('express')();
const http = require('http').Server(app);
const io = require('socket.io')(http);
const port = process.env.PORT || 3000;
const controller = require('./controller');

app.get('/', (req, res) => {
    res.sendFile(__dirname + '/index.html');
});

io.on('connection', (socket) => {
    console.log(socket.conn.remoteAddress, socket.conn.id)
    io.emit('start', { message: `New bot ${socket.conn.id} connected` })
    socket.emit('bot:status', {})
    socket.on("picking:accepted", controller.handlePickingAccepted);
    socket.on("picking:complete", controller.handlePickingComplete);
    socket.on("picking:error", controller.handlePickingError);

    socket.on("putting:accepted", controller.handlePuttingAccepted);
    socket.on("putting:complete", controller.handlePuttingComplete);
    socket.on("putting:error", controller.handlePuttingError);

    socket.on("bot:init", controller.handleBotInit);
    socket.on("bot:status", controller.handleBotStatus);
    socket.on("bot:error", controller.handleBotError);
    socket.on('bot:pathLog', controller.handleBotPathLog);
});

http.listen(port, () => {
    console.log(`Socket.IO server running at http://localhost:${port}/`);
});