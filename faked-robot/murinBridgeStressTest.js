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
        // sendingCount++
        // robotHub.write(`1749486877{"topic":"config","crc":"false"}`)
    }, 1000)
    setTimeout(() => {
        ros2ControlInterval = setInterval(() => {
            sendingCount++
            send_t = Date.now()
            robotHub.write(`150088878{"topic":"ros2_state"}\r\n`)
            console.log(`${dayjs().format('hh:mm:ss.SSS')} ${sendingCount} ==> 150088878{"topic":"ros2_state"}`)
        }, 20)
    }, 2000)
    console.log('rgb hub opened')
});

robotHub.on('data', function (data) {
    const value = String(data)
    messageBufferFromRgbHub += value.trim()
    if (value[value.length - 1] == '\n') {
        receivingCount++
        let receive_t = Date.now()
        console.log(`${dayjs().format('hh:mm:ss.SSS')} ${receivingCount} <== ${messageBufferFromRgbHub} ${receive_t - send_t}`)
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

function msgProcess(msg) {
    const startIdx = msg.indexOf('{')
    const stopIdx = msg.lastIndexOf('}')
    let str = msg.substr(startIdx, stopIdx - startIdx + 1)
    try {
        const obj = JSON.parse(str)
        switch (obj.topic) {
            case 'ros2_state':
                send_t = Date.now()
                sendingCount++
                const send_msg = `1486596010{"topic":"ros2_control","vel":[4.03,4.02,4.04,4.03]}\r\n`
                robotHub.write(send_msg)
                // robotHub.write(`1768921197{"topic":"ros2_control","velocity":[2.03,2.02,2.04,2.03]}\r\n`)
                console.log(`${dayjs().format('hh:mm:ss.SSS')} ${sendingCount} ==> ${send_msg.trim()}`)
                break
            case 'ros2_control':
                break

            default:
                return
        }
    }

    catch (error) {
    }
}