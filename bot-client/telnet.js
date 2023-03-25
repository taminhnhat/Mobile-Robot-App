var net = require('net')
const logger = require('./logger/logger')
const port = 8080;

var client = new net.Socket()
client.connect(port, '192.168.1.42')

const plan = ['RUN:20:10:10/\n', 'RUN:20:20:10/\n', 'RUN:10:20:10/\n', 'RUN:10:15:10/\n']
let step = 1

client.on('connect', () => {
    logger.info('Connected to simulator')
    // client.write('RUN:10:10:10/\n')
})

client.on('close', () => {
    logger.warn('Connection closed')
    setTimeout(() => {
        client.connect(port, '192.168.1.42')
    }, 2000)
})

client.on('data', function (data) {
    logger.info('Received: ' + data)
    if (step <= plan.length) {
        // client.write(plan[step - 1])
        step += 1
    }
    else step = 1
})


client.on('error', err => {
    logger.error('telnet error', { err: err })
})


module.exports = client