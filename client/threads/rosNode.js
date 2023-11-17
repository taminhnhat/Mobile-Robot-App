const { parentPort, workerData } = require('worker_threads');
const { exec, execSync, spawn } = require('child_process');

let nodes = []
let isRefreshed = false
let count = 0

parentPort.on('error', err => console.log(err));
parentPort.on('message', msg => {
    if (msg == 'run') isRefreshed = false
});

const debug = false

const process = () => {
    if (!isRefreshed) {
        count++
        if (debug) console.log('ros node thread: ', count)
        exec('ros2 node list', (err, stdout, stderr) => {
            if (err) {
                console.error(err)
            } else {
                // the *entire* stdout and stderr (buffered)
                if (stdout) {
                    let d = stdout.trim()
                    nodes = d.split('\n')
                    parentPort.postMessage(nodes)
                }
                if (stderr) console.log(`stderr: ${stderr}`)

            }
        })
        isRefreshed = true
    }
    setTimeout(process, 3000)
}

process()