const { parentPort, workerData } = require('worker_threads');
const { exec, execSync } = require('child_process');

let topics = []
let count = 0

parentPort.on('error', err => console.log(err));
parentPort.on('message', msg => {
    if (msg == 'run') isRefreshed = false
});

const debug = false

const process = () => {
    count++
    if (debug) console.log('ros topic thread: ', count)
    exec('ros2 topic list', (err, stdout, stderr) => {
        if (err) {
            console.error(err)
        } else {
            // the *entire* stdout and stderr (buffered)
            if (stdout) {
                let d = stdout.trim()
                topics = []
                let topicsName = d.split('\n')
                // console.log(topicsName)
                topicsName.forEach(async (tp, tpi) => {
                    let out = execSync(`ros2 topic info ${tp}`)
                    // console.log(String(out))
                    if (err) console.log(err)
                    topics.push({ name: tp, type: String(out).trim().split(' ')[1] })

                    if (tpi >= topicsName.length - 1) {
                        parentPort.postMessage(topics)
                    }
                })
            }
            if (stderr) console.log(`stderr: ${stderr}`)

            setTimeout(process, 3000)
        }
    })
}

process()