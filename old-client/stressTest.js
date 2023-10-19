const driver = require('./driver')
const logger = require('./logger/logger');

msgGenerate = () => {
    const v = Math.random() * 0.4 + 0.4
    const msg = JSON.stringify({
        topic: "control",
        linear: [v, 0, 0],
        angular: [0, 0, 0],
        timeout: 2000
    }) + '\r\n'
    return msg
}
driver.port.on('open', function () {
    setInterval(() => {
        const msg = msgGenerate()
        driver.write(msg)
        console.log(msg)
    }, 1000)
});
