const axios = require('axios');
const { generateStock } = require('./stockGenerate');

function createBin(_binId, _location, _mapPoint) {
    const stock = generateStock()
    axios.post('http://192.168.1.42:80/api/v1/createBin', {
        binId: _binId,
        location: _location,
        mapPoint: _mapPoint,
        stock: stock
    }, {
        headers: {
            api_key: 'mgw_cEfRlzOgO2EwRe9ha7Ho'
        }
    })
        .then(res => {
            console.log(res.data)
        })
        .catch(err => console.log(err.message))
}

for (let y = 1; y <= 12; y++) {
    for (let x = 4; x <= 12; x++) {
        createBin(`M-${y}-${x - 3}`, `X${x}Y${4 * y}D-1`, `G-${x}-${4 * y - 2}`)
        createBin(`M-${y}-${22 - x}`, `X${x}Y${4 * y}D1`, `G-${x}-${4 * y + 2}`)

        createBin(`M-${y + 12}-${x - 3}`, `X${x + 15}Y${4 * y}D-1`, `G-${x + 15}-${4 * y - 2}`)
        createBin(`M-${y + 12}-${22 - x}`, `X${x + 15}Y${4 * y}D1`, `G-${x + 15}-${4 * y + 2}`)
    }
}