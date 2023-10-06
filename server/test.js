const { io } = require("socket.io-client")
require('dotenv').config({ path: './.env' })
const url = process.env.SERVER_URL || 'http://localhost:3003'
console.log(`connecting to ${url}`)
const socket = io(url)

socket.on('connect', () => console.log('connected', socket.id))
socket.on('disconnect', () => console.log('disconnected', socket.id))
socket.on('error', err => console.log('error', err))
socket.on('tick', t => console.log('tick', t))

socket.on('gamepad', d => {
    console.log('gamepad', d)
})

// socket.on('ros:topic', d => {
//     console.log('ros:topic', d)
// })

