const axios = require('axios')
require('dotenv').config({ path: './stock-server/.env' })
const { generateStock } = require('./stockGenerate')

function createBin(_binId, _locationCoor, _locationCode) {
    const stock = generateStock()
    axios.post(`http://192.168.1.42:${process.env.HTTP_PORT}/api/v1/createBin`, {
        binId: _binId,
        location: _locationCode,
        mapPoint: _locationCoor,
        stock: stock
    }, {
        headers: {
            api_key: process.env.TOKEN_SECRET
        }
    })
        .then(res => {
            console.log(res.data)
        })
        .catch(err => console.log(err.message))
}

for (let y = 1; y <= 12; y++) {
    for (let x = 4; x <= 12; x++) {
        createBin(`M-${y}-${x - 3}`, `X${x}Y${4 * y}P270`, `G-${x}-${4 * y - 2}`)
        createBin(`M-${y}-${22 - x}`, `X${x}Y${4 * y}P90`, `G-${x}-${4 * y + 2}`)

        createBin(`M-${y + 12}-${x - 3}`, `X${x + 15}Y${4 * y}P270`, `G-${x + 15}-${4 * y - 2}`)
        createBin(`M-${y + 12}-${22 - x}`, `X${x + 15}Y${4 * y}P90`, `G-${x + 15}-${4 * y + 2}`)
    }
}