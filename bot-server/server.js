const server = require('./app')
const fetch = require('node-fetch')
const io = require('socket.io')(server)
const controller = require('./controller')
require('dotenv').config({ path: './bot-server/.env' })
const port = Number(process.env.HTTP_PORT) || 3002

// connect db
const mongoose = require('mongoose')
mongoose.connect(process.env.DATABASE_URL, { useNewUrlParser: true })
    .catch(err => {
        console.log(err)
    })

const db = mongoose.connection
db.on('error', (error) => console.error(error))
db.once('open', () => console.log('Connected to Database'))

io.on('connection', (socket) => {
    console.log(socket.conn.remoteAddress, socket.conn.id)
    io.emit('start', { message: `New bot ${socket.conn.id} connected` })

    async function handleOrderAdd(dataFromSocket) {
        socket.emit('order:add', {
            message: 'accepted'
        })
        console.log('dataFromSocket:', dataFromSocket)
        dataFromSocket.items.forEach(async (eachItem, itemIndex) => {
            try {
                const searchRespond = await fetch(`http://192.168.1.42:3001/api/v1/searchItem?productId=${eachItem.id}`, {
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
                    out.push({ binId: eachBin.binId, mapPoint: eachBin.mapPoint, qty: eachBin.stock[0].qty })
                })
                eachItem.location = out
                if (itemIndex == dataFromSocket.items.length - 1) {
                    console.log('result:', JSON.stringify(dataFromSocket.items))
                    socket.emit('order:info', {
                        orderId: dataFromSocket.orderId,
                        status: 'accepted',
                        items: dataFromSocket.items
                    })
                    //
                    const routingPlan = []
                }
            }
            catch (err) {
                console.log(err.message)
            }
        })
    }

    function generateRoutingPlan(items) {
        let planA = []
        items.forEach(eachItem => {
            planA.push[{
                id: eachItem.id,
                qty: eachItem.qty,
                location: eachItem.location[0].mapPoint
            }]
        })
        console.log(planA)
        return planA
    }

    /**
     * 
     * @param {Array} botStopList List of stop points for robot to colect items
     */
    function pathPlanning(routingPlan) {
        //
    }

    socket.on('order:add', handleOrderAdd)

    socket.emit('bot:status', {})
    socket.on("picking:accepted", controller.handlePickingAccepted)
    socket.on("picking:complete", controller.handlePickingComplete)
    socket.on("picking:error", controller.handlePickingError)

    socket.on("putting:accepted", controller.handlePuttingAccepted)
    socket.on("putting:complete", controller.handlePuttingComplete)
    socket.on("putting:error", controller.handlePuttingError)

    socket.on("bot:init", controller.handleBotInit)
    socket.on("bot:status", controller.handleBotStatus)
    socket.on("bot:error", controller.handleBotError)
    socket.on('bot:pathLog', controller.handleBotPathLog)
});

server.listen(port, () => {
    console.log(`Socket.IO server running at http://localhost:${port}/`);
});