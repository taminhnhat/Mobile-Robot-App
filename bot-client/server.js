const { io } = require("socket.io-client")
const socket = io('http://192.168.1.42:3002')
const driver = require('./telnet')
const plan = ['RUN:20:10:10/\n', 'RUN:20:20:10/\n', 'RUN:10:20:10/\n', 'RUN:10:15:10/\n']

let step = 1
driver.write('RUN:10:10:10/\n')
driver.on('data', function (data) {
    console.log('Received: ' + data)
    if (step <= plan.length) {
        driver.write(plan[step - 1])
        step += 1
    }
    // else driver.destroy()
})

driver.on('close', function () {
    console.log('Connection closed')
})

driver.on('error', err => {
    console.log(err)
})

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