var net = require('net')
const port = 8080;

var client = new net.Socket()
client.connect(port, '192.168.1.42')

module.exports = client