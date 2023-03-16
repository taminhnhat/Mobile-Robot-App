const server = require('./app')
const io = require('socket.io')(server)
const event = require('./middlewares/event')
require('dotenv').config({ path: './bot-server/.env' })
const port = Number(process.env.HTTP_PORT) || 3002

// connect db
const mongoose = require('mongoose')
mongoose.connect(process.env.DATABASE_URL, { useNewUrlParser: true })
    .catch(err => {
        console.log(err)
    })

const robotCtrl = require('./robots.js')
const core = require('./core')

const db = mongoose.connection
db.on('error', (error) => console.error(error))
db.once('open', () => console.log('Connected to Database'))

const onConnection = (socket) => {
    const msg = `New bot ${socket.conn.id}|${socket.conn.remoteAddress} connected`
    console.log(msg)
    io.emit('start', { message: msg })

    socket.on('robot:connect', robotCtrl.onConnect)
    socket.on('robot:info', robotCtrl.onInfo)
    socket.on('robot:message', robotCtrl.onMessage)

    event.on('robot:plan', robotCtrl.onPlan)
}
event.on('core:addOrder', core.onAddOrder)

io.on('connection', onConnection);

server.listen(port, () => {
    console.log(`Robot server running at http://localhost:${port}/`);
});