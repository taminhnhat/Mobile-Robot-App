const fetch = require('node-fetch')
const fs = require('fs')

let orders = []

const onAddOrder = d => {
    try {
        d.items.forEach(async (item, itemIdx) => {
            // search each item in stock
            const res = await fetch(`http://192.168.1.42:3001/api/v1/searchItem?productId=${item.id}`, {
                method: 'GET',
                headers: {
                    api_key: process.env.TSOCK_TOKEN
                }
            })
            let locations = await res.json()
            locations = locations.data
            // console.log(`${searchRespond.status} ${searchRespond.statusText}`, respondBody)
            locations.filter(location => {
                return location.stock[0].qty >= item.qty
            })
            item.location = locations
            if (itemIdx == d.items.length - 1) {
                fs.writeFile('location.json', JSON.stringify(d.items, null, 4), () => { })
            }
        })
    }
    catch (err) {
        console.log('error', err)
    }
}

module.exports = {
    onAddOrder
}