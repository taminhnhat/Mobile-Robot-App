const fs = require('fs');
const { spawn, fork } = require('child_process');
let logger = require('./logger/logger')

// const event = require('./event');

const read_pipe_path = '/tmp/user_cmd';
let readfifo = spawn('mkfifo', [read_pipe_path]);

readfifo.on('exit', function (status) {
    console.log('pipe opened');
    const fileHandle = fs.openSync(read_pipe_path, 'r+');
    let fifoRs = fs.createReadStream(null, { fd: fileHandle });

    // Handle Reading pipe event
    fifoRs.on('data', data => {
        data = String(data).trim();
        const dataArray = data.split(':');
        const header = dataArray[0];
        switch (header) {
            case 'on':
                break;
            case 'off':
                break;
            default:
                break;
        }
    });

    fifoRs.on('ready', function (err) {
        logger.debug({ message: 'Reading pipe is ready', location: FILE_NAME });
    });

    fifoRs.on('open', function (err) {
        logger.debug({ message: 'Reading pipe opened', location: FILE_NAME });
    });

    fifoRs.on('close', function (err) {
        logger.debug({ message: 'Reading pipe closed', location: FILE_NAME });
    });
});


/**
 * 
 * @param {*} size size of check string
 * @returns string type of an interger
 */
function generateCheck(size) {
    let val = '';
    for (let i = 0; i < size; i++) {
        val += Math.floor(Math.random() * 10);
    }
    return val;
}