const { io } = require("socket.io-client")
const socket = io('http://192.168.1.42:3002')
const logger = require('./logger/logger')
require('dotenv').config({ path: './bot-client/.env' })
require('./driver')

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