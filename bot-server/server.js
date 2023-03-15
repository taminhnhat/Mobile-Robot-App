const server = require('./app')
const fetch = require('node-fetch')
const io = require('socket.io')(server)
const controller = require('./controller')
const event = require('./middlewares/event')
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

    async function handleOrderAdd(data) {
        console.log('dataFromSocket:', data)
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

    event.on('order:add', handleOrderAdd)
});

server.listen(port, () => {
    console.log(`Socket.IO server running at http://localhost:${port}/`);
});