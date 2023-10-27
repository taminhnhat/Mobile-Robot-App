const { io } = require("socket.io-client")
const { exec, execSync } = require('child_process');
const { Worker } = require('worker_threads');
require('dotenv').config({ path: './.env' })
const url = process.env.SERVER_URL || 'http://localhost:3003'
console.log(`connecting to ${url}`)
const socket = io(url)

socket.on('connect', () => console.log('connected', socket.id))
socket.on('disconnect', () => console.log('disconnected', socket.id))
socket.on('error', err => console.log('error', err))
socket.on('tick', t => console.log('tick', t))

let robotMonitorData = {
    nodes: [],
    topics: [],
    services: [],
    nodesUpdated: false,
    topicsUpdated: false,
}

getRosInfo()
function getRosInfo() {
    let wk = new Worker(__dirname + '/threads/rosNode.js')
    let wk2 = new Worker(__dirname + '/threads/rosTopic.js')

    wk2.on('message', (msg) => {
        robotMonitorData.topics = msg
        robotMonitorData.topicsUpdated = true
        if (rosInfoCheck()) {
            wk.postMessage('run')
            wk2.postMessage('run')
        }
    });

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
    // setTimeout(getRosState, 2000)
}

getServiceInfo()
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

const fs = require('fs');
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