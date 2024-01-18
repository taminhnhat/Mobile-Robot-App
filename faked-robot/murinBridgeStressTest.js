const { SerialPort } = require('serialport');
const CRC = require('crc')
require('dotenv').config()
const dayjs = require('dayjs')

let isRgbHubOpen = false
const usbPath = process.env.USB_PORT || '/dev/ttyUSB0'

let messageBufferFromRgbHub = ''

let reconnectHubInterval
let ros2ControlInterval
let sendingCount = 0
let receivingCount = 0

//  NEED TO CONFIG SERIAL PORT FIRST, READ 'README.md'
const robotHub = new SerialPort({
    path: usbPath,
    baudRate: Number(process.env.USB_BAUDRATE) || 115200,
    autoOpen: true
});

let send_t = Date.now()

robotHub.on('open', function () {
    isRgbHubOpen = true
    clearInterval(reconnectHubInterval)
    setTimeout(() => {
        robotHub.write(`1749486877{"topic":"config","crc":"false"}`)
    }, 1000)
    setTimeout(() => {
        ros2ControlInterval = setInterval(() => {
            sendingCount++
            // robotHub.write(`{"topic":"ros2_control","velocity":[4.03562,4.02,4.043256324,4.034566],"timeout":1000}\r\n`)
            send_t = Date.now()
            robotHub.write(`150088878{"topic":"ros2_state"}\r\n`)
        }, 20)
    }, 2000)
    console.log('rgb hub opened')
});

robotHub.on('data', function (data) {
    receivingCount++
    const value = String(data)
    messageBufferFromRgbHub += value.trim()
    if (value[value.length - 1] == '\n') {
        let receive_t = Date.now()
        console.log(`sent:${sendingCount}   received:${receive_t - send_t}   ${dayjs().format('hh:mm:ss.SSS')} <<< ${messageBufferFromRgbHub}`)
        // msgProcess(messageBufferFromRgbHub)
        messageBufferFromRgbHub = ''
    }
});

robotHub.on('close', () => {
    console.log('Rgb hub closed')
    reconnectHubInterval = setInterval(() => {
        robotHub.open((err) => {
            //
        });
    }, 10000)
});

robotHub.on('error', (err) => {
    console.log('Rgb hub error', { error: err })
});

function msgProcess(msg) {
    //
}