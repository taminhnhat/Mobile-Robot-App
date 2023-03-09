var net = require('net')

var client = new net.Socket()
client.connect(9600, '192.168.1.42', function () {
    console.log('Connected')
    client.write('RUN:20:10:10/\n')
})

client.on('data', function (data) {
    console.log('Received: ' + data)
    client.destroy() // kill client after server's response
})

client.on('close', function () {
    console.log('Connection closed')
})

client.on('error', err => {
    console.log(err)
})