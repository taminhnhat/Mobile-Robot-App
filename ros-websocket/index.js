const { io } = require("socket.io-client")
require('dotenv').config({ path: './.env' })
const url = process.env.SERVER_URL || 'http://192.168.1.42:3003'
console.log(`connecting to ${url}`)
const socket = io(url)

socket.on('connect', () => console.log('connected', socket.id))
socket.on('disconnect', () => console.log('disconnected', socket.id))
socket.on('error', err => console.log('error', err))
socket.on('tick', t => console.log('tick', t))


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

const rclnodejs = require('rclnodejs')
rclnodejs.init()
    .then(() => {
        const rclTime = new rclnodejs.Clock()

        const node = rclnodejs.createNode('subscription_example_node');
        console.log('created node')

        // node.createSubscription('geometry_msgs/msg/Twist', 'cmd_vel_joy', (msg) => {
        //     console.log(`Received message: ${typeof msg}`, msg);
        //     socket.emit('ros:topic', {
        //         topic: 'cmd_vel_joy',
        //         message: msg
        //     })
        // });
        const ws_velocity_publisher = node.createPublisher('geometry_msgs/msg/Twist', 'ws_vel');
        const imu_publisher = node.createPublisher('sensor_msgs/msg/Imu', 'imu');
        const battery_publisher = node.createPublisher('sensor_msgs/msg/Imu', 'test/battery');

        socket.on('ros:topic', d => {
            console.log(d)
            if (d.topic == 'ws_vel')
                ws_velocity_publisher.publish({
                    linear: { x: d.data.linear[0], y: d.data.linear[1], z: d.data.linear[2] },
                    angular: { x: d.data.angular[0], y: d.data.angular[1], z: d.data.angular[2] },
                })
            console.log('ros:topic', d)
        })

        fifoRs.on('data', d => {
            const t = rclTime.now().secondsAndNanoseconds
            const dStr = String(d).substring(d.lastIndexOf('{'), d.lastIndexOf('}') + 1)
            const dJson = JSON.parse(dStr)
            let dPro = JSON.parse(JSON.stringify(dJson))
            delete dPro.topic
            socket.emit('ros:topic', {
                topic: dJson.topic,
                data: dPro,
            })

            if (dJson.topic == 'ros2_state') {
                if (dJson.gyr != undefined && dJson.acc != undefined)
                    imu_publisher.publish({
                        header: { frame_id: 'imu_link', stamp: { sec: t.seconds, nanosec: t.nanoseconds } },
                        orientation: { x: dJson.ori[0], y: dJson.ori[1], z: dJson.ori[2], w: dJson.ori[3] },
                        angular_velocity: { x: dJson.gyr[0], y: dJson.gyr[1], z: dJson.gyr[2] },
                        linear_acceleration: { x: dJson.acc[0], y: dJson.acc[1], z: dJson.acc[2] },
                    })
            }
        })

        rclnodejs.spin(node);
    })
    .catch((e) => {
        console.log(e);
    })