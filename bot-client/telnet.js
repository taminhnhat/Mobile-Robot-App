var net = require('net')

var client = new net.Socket()
client.connect(9600, '192.168.1.42', function () {
    console.log('Connected to simulator')
})

module.exports = client