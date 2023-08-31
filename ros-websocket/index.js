const { io } = require("socket.io-client")
const socket = io('http://192.168.1.42:3003')

socket.on('connect', () => console.log('connected', socket.id))
socket.on('disconnect', () => console.log('disconnected', socket.id))
socket.on('error', err => console.log('error', err))
socket.on('tick', t => console.log('tick', t))

const rclnodejs = require('rclnodejs')
rclnodejs.init()
    .then(() => {

        const node = rclnodejs.createNode('subscription_example_node');
        console.log('created node')

        node.createSubscription('geometry_msgs/msg/Twist', 'cmd_vel_joy', (msg) => {
            console.log(`Received message: ${typeof msg}`, msg);
            socket.emit('ros:topic', {
                topic: 'cmd_vel_joy',
                message: msg
            })
        });
        const publisher = node.createPublisher('std_msgs/msg/String', 'ws_vel');
        socket.on('ros:topic', d => {
            publisher.publish(d);
            console.log('ros:topic', d)
        })

        rclnodejs.spin(node);
    })
    .catch((e) => {
        console.log(e);
    })