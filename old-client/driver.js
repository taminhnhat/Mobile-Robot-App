/*************
 * This file used to interact serial device on USB port
 * Scanners are configured
 */
require('dotenv').config({ path: './.env' });
const { SerialPort } = require('serialport');

const logger = require('./logger/logger');
const rgbHubCycleInMilis = Number(process.env.RGB_HUB_SERIAL_CYCLE) || 100;
const rgbHubDebugMode = process.env.RGB_DEBUG_MODE;

let messageBufferFromRgbHub = ''

let messageInQueue = []
let lastCallInMilis = 0
let reconnectHubInterval


//  NEED TO CONFIG SERIAL PORT FIRST, READ 'README.md'
const serialDevice = new SerialPort({ path: process.env.DRIVER_PATH, baudRate: Number(process.env.DRIVER_BAUDRATE) || 115200, autoOpen: false });

serialDevice.on('open', function () {
  clearInterval(reconnectHubInterval)
  logger.info({ message: 'rgb hub opened' })
});

serialDevice.on('data', function (data) {
  const value = String(data).trim()
  logger.debug({ message: value })
});

serialDevice.on('close', () => {
  logger.warn('Rgb hub closed')
  reconnectHubInterval = setInterval(() => {
    serialDevice.open((err) => {
      //
    });
  }, 10000)
});

serialDevice.on('error', (err) => {
  logger.error('Rgb hub error', { error: err })
});

/**
 *
 * @param {String} message
 */
function queue(message) {
  messageInQueue.push(message)
  next()
}

function next() {
  let loadingQueue = messageInQueue
  // messageInQueue.forEach((value, index) => {
  //   //
  // })
  // while (loadingQueue.length > 0) {
  // }
  const messageToRgbHub = loadingQueue[loadingQueue.length - 1]
  if (messageToRgbHub == undefined) {
    return
  }
  const presentInMilis = Date.now()
  const delayTimeInMilis = lastCallInMilis + rgbHubCycleInMilis - presentInMilis
  if (delayTimeInMilis > 0) {
    setTimeout(() => {
      serialDevice.write(messageToRgbHub, (err, res) => {
        if (err) logger.error({ message: 'Cannot write to rgb hub', error: err });
        if (rgbHubDebugMode == 'true');
      });
    }, delayTimeInMilis)
    lastCallInMilis += rgbHubCycleInMilis
  }
  else {
    setImmediate(() => {
      serialDevice.write(messageToRgbHub, (err, res) => {
        if (err) logger.error({ message: 'Cannot write to rgb hub', error: err });
        if (rgbHubDebugMode == 'true');
      });
    })
    lastCallInMilis = presentInMilis
  }
  messageInQueue.pop()
}

serialDevice.open((err) => {
  if (err) logger.error({ message: 'Can not open driver', error: err });
  reconnectHubInterval = setInterval(() => {
    serialDevice.open((err) => {
      //
    });
  }, 10000)
});

module.exports = { port: serialDevice, write: queue }