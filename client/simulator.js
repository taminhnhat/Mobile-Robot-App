let net = require('net')
require('dotenv').config({ path: './bot-client/.env' })
const port = process.env.SIMULATOR_PORT;
let client = new net.Socket()

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
})

client.connect(port, '192.168.1.42')

