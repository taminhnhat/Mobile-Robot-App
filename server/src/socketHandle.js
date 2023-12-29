const io = require('socket.io')(server)
io.on('connection', onConnection);

const onConnection = (socket) => {
    const msg = `New bot ${socket.conn.id}|${socket.conn.remoteAddress} connected`
    logger.info(msg)
    io.emit('start', { message: msg })

    // socket.on('robot:connect', robotCtrl.onConnect)
    socket.on('ros:topic', d => {
        console.log(d.topic, d.data)
    })
    // socket.on('robot:message', robotCtrl.onMessage)
    setInterval(() => {
        // socket.emit('ros:topic', 'control')
    }, 1000)
}

module.exports = onConnection