const app = require('./app')
const http = require("http")
const server = http.createServer(app)
const io = require('socket.io')(server)
// const event = require('./middlewares/event')
// require('dotenv').config({ path: './bot-server/.env' })
const port = Number(process.env.HTTP_PORT) || 3003
const logger = require('./logger/logger')
// const { robotPlot } = require('./robotManagement/map_2')

// // connect db
// const mongoose = require('mongoose')
// mongoose.connect(process.env.DATABASE_URL, { useNewUrlParser: true })
//     .catch(err => {
//         logger.info(err)
//     })

// const robotCtrl = require('./robots.js')
// const core = require('./core')

// const db = mongoose.connection
// db.on('error', (error) => console.error(error))
// db.once('open', () => logger.info('Connected to Database'))

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

app.get('/', (req, res) => {
    res.sendFile(__dirname + '/controller.html');
})
app.get('/joy', (req, res) => {
    res.sendFile(__dirname + '/joy.js');
})

io.on('connection', onConnection);

server.listen(port, () => {
    logger.info(`Robot server running at http://localhost:${port}/`);
});