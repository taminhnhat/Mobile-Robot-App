const { io } = require("socket.io-client")
const { exec, execSync, execFile, spawn } = require('child_process');
const { Worker } = require('worker_threads');
const kill = require('tree-kill')
require('dotenv').config({ path: './.env' })
const url = process.env.SERVER_URL || 'http://localhost:3003'
console.log(`connecting to ${url}`)
const socket = io(url)

socket.on('connect', () => console.log('connected', socket.id))
socket.on('disconnect', () => console.log('disconnected', socket.id))
socket.on('error', err => console.log('error', err))
socket.on('tick', t => console.log('tick', t))
// socket.on('robot:camera:start', startCamera)
// socket.on('robot:camera:stop', stopCamera)

let robotMonitorData = {
    nodes: [],
    topics: [],
    services: [],
    nodesUpdated: false,
    topicsUpdated: false,
}

// getRosInfo()
function getRosInfo() {
    let wk = new Worker(__dirname + '/threads/rosNode.js')
    let wk2 = new Worker(__dirname + '/threads/rosTopic.js')

    wk.on('error', err => console.log(err));

    wk.on('exit', (code) => {
        if (code !== 0) console.error(new Error(`Worker stopped Code ${code}`))
    });

    wk.on('message', (msg) => {
        robotMonitorData.nodes = msg
        robotMonitorData.nodesUpdated = true
        if (rosInfoCheck()) {
            wk.postMessage('run')
            wk2.postMessage('run')
        }
    });

    wk2.on('error', err => console.log(err));

    wk2.on('exit', (code) => {
        if (code !== 0) console.error(new Error(`Worker stopped Code ${code}`))
    });

    wk2.on('message', (msg) => {
        robotMonitorData.topics = msg
        robotMonitorData.topicsUpdated = true
        if (rosInfoCheck()) {
            wk.postMessage('run')
            wk2.postMessage('run')
        }
    });
    // setTimeout(getRosState, 2000)
}

// getServiceInfo()
function getServiceInfo() {
    let wk = new Worker(__dirname + '/threads/userService.js')

    wk.on('message', (msg) => {
        robotMonitorData.services = msg
    });

    wk.on('error', err => console.log(err));

    wk.on('exit', (code) => {
        if (code !== 0) console.error(new Error(`Worker stopped Code ${code}`))
    });
}

function rosInfoCheck() {
    if (robotMonitorData.nodesUpdated === true && robotMonitorData.topicsUpdated === true) {
        robotMonitorData.nodesUpdated = false
        robotMonitorData.topicsUpdated = false
        socket.emit('ros:monitor', robotMonitorData)
        return true
    }
    else return false
}

let cameraThread
let rsChild

function startCamera() {
    console.log('start realsense')
    const child = spawn('./threads/realsense.sh')
    child.stdout.on('data', data => {
        console.log(`stdout:\n${data}`);
        setTimeout(() => {
            // kill(child.pid)
            // child.kill('SIGINT')
            process.kill(child.pid, 'SIGINT')
        }, 5000)
    });
    child.stderr.on('data', data => {
        console.error(`stderr: ${data}`);
    });
    child.on('exit', code => {
        console.log(`process stop with code ${code}`)
    })
    console.log(child.pid)
    // rsChild = exec('./threads/realsense.sh', { killSignal: 'SIGTERM' }, (err, stdout, stderr) => {
    // if (err) {
    //     console.error(`error: ${error.message}`);
    //     return;
    // }

    // if (stderr) {
    //     console.error(`stderr: ${stderr}`);
    //     return;
    // }
    // console.log(`stdout:\n${stdout}`);

    // console.log(rsChild)
    // setInterval(() => console.log(rsChild.pid), 1000)
    // cameraThread = new Worker(__dirname + '/threads/cameraService.js')
    // console.log('start camera thread, id: ', cameraThread.threadId)

    // cameraThread.on('message', (msg) => {
    //     console.log(msg)
    // });

    // cameraThread.on('error', err => console.log(err));

    // cameraThread.on('exit', (code) => {
    //     if (code !== 0) console.error(new Error(`Worker stopped Code ${code}`))
    // });
}
function stopCamera() {
    rsChild.kill('SIGINT')
    //     cameraThread.terminate();
    // console.log('topic stop camera')
}

const fs = require('fs');
const { Console } = require("console");
const { stderr } = require("process");
const fileHandle = fs.openSync('/tmp/ros2_control_out', 'r+');
let fifoRs = fs.createReadStream(null, { fd: fileHandle });

fifoRs.on('ready', function (err) {
    console.log('Reading pipe is ready')
});

fifoRs.on('open', function (err) {
    console.log('Reading pipe opened')
});

fifoRs.on('close', function (err) {
    console.log('Reading pipe closed')
});

fifoRs.on('data', d => {
    const dStr = String(d).substring(d.lastIndexOf('{'), d.lastIndexOf('}') + 1)
    const dJson = JSON.parse(dStr)
    // console.log('pipe out >>>')
    // console.log(dJson.vel)
    let dPro = JSON.parse(JSON.stringify(dJson))
    delete dPro.topic
    socket.emit('ros:topic', {
        topic: dJson.topic,
        data: dPro,
    })
})