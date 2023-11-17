const { parentPort, workerData } = require('worker_threads');
const { exec, execSync } = require('child_process');

parentPort.on('error', err => console.log(err));
parentPort.on('message', msg => {

});

const process = () => {
    try {
        execSync('./scripts/realsense.sh')
    }
    catch (err) {
        console.log(err)
        return
    }
}

process()