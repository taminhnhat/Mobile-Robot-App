const { io } = require("socket.io-client")
const socket = io('http://192.168.1.42:3003')
const rclnodejs = require('rclnodejs')

socket.on('connect')

rclnodejs.init()
    .then(() => {
        const node = rclnodejs.createNode('subscription_example_node');

        node.createSubscription('geometry_msgs/msg/Twist', 'cmd_vel_joy', (msg) => {
            console.log(`Received message: ${typeof msg}`, msg);
        });

        rclnodejs.spin(node);
    })
    .catch((e) => {
        console.log(e);
    });