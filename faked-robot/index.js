const { SerialPort } = require('serialport');
const CRC = require('crc')
require('dotenv').config()

let isRgbHubOpen = false
const usbPath = process.env.USB_PORT || '/dev/ttyUSB0'

let messageBufferFromRgbHub = ''

let messageInQueue = []
let lastCallInMilis = 0
let reconnectHubInterval

//  NEED TO CONFIG SERIAL PORT FIRST, READ 'README.md'
const robotHub = new SerialPort({
    path: usbPath,
    baudRate: 460800,
    autoOpen: true
});

robotHub.on('open', function () {
    isRgbHubOpen = true
    clearInterval(reconnectHubInterval)
    console.log('rgb hub opened')
});

robotHub.on('data', function (data) {
    const value = String(data)
    messageBufferFromRgbHub += value.trim()
    if (value[value.length - 1] == '\n') {
        console.log('<<<', messageBufferFromRgbHub)
        msgProcess(messageBufferFromRgbHub)
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


let vel = [0, 0, 0, 0]
let pos = [0, 0, 0, 0]
let ori = [0, 0, 0, 0]
let gyr = [0, 0, 0]
let acc = [0, 0, 0]
let lastCall = Date.now()
function msgProcess(msg) {
    const dStr = msg.substring(msg.lastIndexOf('{'), msg.lastIndexOf('}') + 1)
    const dJson = JSON.parse(dStr)
    // console.log('==>', dJson.topic)
    switch (dJson.topic) {
        case 'ros2_control':
            const instantCall = Date.now()
            vel.forEach((v, i) => {
                pos[i] = Number((pos[i] + v * (instantCall - lastCall) / 1000).toFixed(2))
            })
            lastCall = instantCall
            vel = dJson.velocity
            m = JSON.stringify({
                topic: 'ros2_control',
                status: 'ok'
            })
            res = robotRes(m)
            robotHub.write(res)
            break;
        case 'ros2_state':
            // m = JSON.stringify({
            //     topic: 'ros2_state',
            //     velocity: vel,
            //     position: pos,
            //     battery: 16.5,
            // })
            m = JSON.stringify({
                tp: 'ros2_state',
                vel: vel,
                pos: pos,
                bat: 16.5,
                tem: 30.0,
                ori: ori,
                gyr: gyr,
                acc: acc,
            })
            res = robotRes(m)
            robotHub.write(res)
            // console.log('>>>', res.trim())
            break;
        default:
            break;
    }
}

function robotRes(msg) {
    let cs = CRC.crc32(msg)
    let res = cs + msg + '\r\n'
    return res
}