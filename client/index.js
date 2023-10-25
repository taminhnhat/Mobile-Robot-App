const { io } = require("socket.io-client")
require('dotenv').config({ path: './.env' })
const url = process.env.SERVER_URL || 'http://localhost:3003'
console.log(`connecting to ${url}`)
const socket = io(url)

socket.on('connect', () => console.log('connected', socket.id))
socket.on('disconnect', () => console.log('disconnected', socket.id))
socket.on('error', err => console.log('error', err))
socket.on('tick', t => console.log('tick', t))

const { exec, execSync } = require('child_process');
let rosMonitorData = {
    nodes: [],
    topics: []
}
setInterval(() => exec('ros2 node list', (err, stdout, stderr) => {
    if (err) {
        console.error(err)
    } else {
        // the *entire* stdout and stderr (buffered)
        if (stdout) {
            // console.log(`stdout: ${stdout}`)
            let d = stdout.trim()
            let nodesName = d.split('\n')
            rosMonitorData.nodes = nodesName
        }
        if (stderr) console.log(`stderr: ${stderr}`)
    }
}), 1000)

function getRosState() {
    exec('ros2 topic list', (err, stdout, stderr) => {
        if (err) {
            console.error(err)
        } else {
            // the *entire* stdout and stderr (buffered)
            if (stdout) {
                // console.log(`stdout: ${stdout}`)
                let d = stdout.trim()
                rosMonitorData.topics = []
                let topicsName = d.split('\n')
                topicsName.forEach(async (tp) => {
                    let out = execSync(`ros2 topic info ${tp}`)
                    // console.log(String(out))
                    if (err) console.log(err)
                    rosMonitorData.topics.push({ name: tp, type: String(out).trim().split(' ')[1] })
                })
            }
            if (stderr) console.log(`stderr: ${stderr}`)
            socket.emit('ros:monitor', rosMonitorData)
        }
    })
    setTimeout(getRosState, 1000)
}

getRosState()

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
    // console.log(dJson.vel)
    let dPro = JSON.parse(JSON.stringify(dJson))
    delete dPro.topic
    socket.emit('ros:topic', {
        topic: dJson.topic,
        data: dPro,
    })
})