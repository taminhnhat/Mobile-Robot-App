const { io } = require("socket.io-client")
const socket = io('http://192.168.1.42:3000')

socket.on("start", (data) => {
    console.log(data)
});
socket.on('bot:status', () => {
    socket.emit('bot:status', { status: '' })
})
socket.on('bot:pathLog', () => {
    Socket.emit('bot:pathLog', { id: 'Y-004', des: 'G-2-34', dir: '90', path: [], pos: 'G-2-12', cor: '2200:12340:89' })
})
socket.emit('bot:init', { id: 'Y-004' })