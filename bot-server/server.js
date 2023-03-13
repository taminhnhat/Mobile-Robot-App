const app = require('express')()
const http = require('http').Server(app)
const fetch = require('node-fetch')
const io = require('socket.io')(http)
require('dotenv').config()
const port = process.env.PORT || 3002;
const controller = require('./controller')

app.get('/', (req, res) => {
    res.sendFile(__dirname + '/index.html')
});

io.on('connection', (socket) => {
    console.log(socket.conn.remoteAddress, socket.conn.id)
    io.emit('start', { message: `New bot ${socket.conn.id} connected` })

    socket.on('order:add', handleOrderAdd)
    async function handleOrderAdd(dataFromSocket) {
        socket.emit('order:add', {
            message: 'accepted'
        })
        console.log('dataFromSocket:', dataFromSocket)
        dataFromSocket.items.forEach(async (eachItem, itemIndex) => {
            try {
                const searchRespond = await fetch(`http://192.168.1.42:80/api/v1/searchItem?productId=${eachItem.id}`, {
                    method: 'GET',
                    headers: {
                        api_key: process.env.TOKEN_SECRET
                    }
                })
                const respondBody = await searchRespond.json()
                console.log(`${searchRespond.status} ${searchRespond.statusText}`, respondBody)
                let out = []
                respondBody.data.forEach(eachBin => {
                    // if (eachBin.stock[0].qty >= eachItem.qty)
                    out.push({ binId: eachBin.binId, qty: eachBin.stock[0].qty })
                })
                eachItem.location = out
                console.log(eachItem)
                if (itemIndex == dataFromSocket.items.length - 1)
                    console.log('result:', JSON.stringify(dataFromSocket.items))
            }
            catch (err) {
                console.log(err.message)
            }
        })
    }

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