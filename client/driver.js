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
const driver = new SerialPort({ path: process.env.DRIVER_PATH, baudRate: Number(process.env.DRIVER_BAUDRATE) || 115200, autoOpen: false });

driver.on('open', function () {
  clearInterval(reconnectHubInterval)
  logger.info({ message: 'rgb hub opened' })
});

driver.on('data', function (data) {
  const value = String(data).trim()
  logger.debug({ message: value })
});

driver.on('close', () => {
  logger.warn('Rgb hub closed')
  reconnectHubInterval = setInterval(() => {
    driver.open((err) => {
      //
    });
  }, 10000)
});

driver.on('error', (err) => {
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
      driver.write(messageToRgbHub, (err, res) => {
        if (err) logger.error({ message: 'Cannot write to rgb hub', error: err });
        if (rgbHubDebugMode == 'true');
      });
    }, delayTimeInMilis)
    lastCallInMilis += rgbHubCycleInMilis
  }
  else {
    setImmediate(() => {
      driver.write(messageToRgbHub, (err, res) => {
        if (err) logger.error({ message: 'Cannot write to rgb hub', error: err });
        if (rgbHubDebugMode == 'true');
      });
    })
    lastCallInMilis = presentInMilis
  }
  messageInQueue.pop()
}

driver.open((err) => {
  if (err) logger.error({ message: 'Can not open driver', error: err });
  reconnectHubInterval = setInterval(() => {
    driver.open((err) => {
      //
    });
  }, 10000)
});

module.exports = { write: queue }