const { io } = require("socket.io-client")
const socket = io('http://192.168.1.42:3002')
const driver = require('./telnet')
const logger = require('./logger/logger')
require('dotenv').config({ path: './bot-client/.env' })
const port = process.env.SIMULATOR_PORT;
driver.connect(port, '192.168.1.42')

const plan = ['R01:RUN:20:10:10/\n', 'R01:RUN:20:20:10/\n', 'R01:RUN:10:20:10/\n', 'R01:RUN:10:15:10/\n']
let step = 1

socket.on("start", (data) => {
    logger.info(data)
});
socket.on('bot:status', () => {
    socket.emit('bot:status', { status: '' })
})
socket.on('bot:pathLog', () => {
    Socket.emit('bot:pathLog', { id: 'Y-004', des: 'G-2-34', dir: '90', path: [], pos: 'G-2-12', cor: '2200:12340:89' })
})
socket.emit('bot:init', { id: 'Y-004' })


driver.on('close', () => {
    logger.warn('Connection closed')
    setTimeout(() => {
        driver.connect(port, '192.168.1.42')
    }, 2000)
})

driver.on('error', err => {
    logger.error('telnet error', { err: err })
})

driver.on('connect', () => {
    logger.info('Connected to simulator')
    driver.write('R01:RUN:10:10:10/\n')
})

driver.on('data', function (data) {
    data = data.toString().trim()
    logger.info('Received: ' + data)
    const mesParse = data.match(/\w*/g)
    const robotStatus = {
        id: mesParse[0],
        position: {
            x: mesParse[4],
            y: mesParse[6],
            p: mesParse[8]
        }
    }
    socket.emit('robot:status', robotStatus)
    if (step <= plan.length) {
        driver.write(plan[step - 1])
        step += 1
    }
    else step = 1
})