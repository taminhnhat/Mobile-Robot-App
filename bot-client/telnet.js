var net = require('net')

var client = new net.Socket()
client.connect(9600, '192.168.1.42')

const plan = ['RUN:20:10:10/\n', 'RUN:20:20:10/\n', 'RUN:10:20:10/\n', 'RUN:10:15:10/\n']
let step = 1

client.on('connect', () => {
    console.log('Connected to simulator')
    client.write('RUN:10:10:10/\n')
})

client.on('data', function (data) {
    console.log('Received: ' + data)
    if (step <= plan.length) {
        client.write(plan[step - 1])
        step += 1
    }
    else step = 1
})

client.on('close', function () {
    console.log('Connection closed')
    setTimeout(() => {
        client.connect(9600, '192.168.1.42', function (err) {
            if (err)
                console.log(err)
            console.log('Connected to simulator')
        })
    }, 2000)
})


module.exports = client