const fs = require('fs')
const fetch = require('node-fetch')
const event = require('../middlewares/event')
const orderCollection = require('../models/order.model')
const { showMap } = require('../pathPlanning/map')

async function addOrder(req, res) {
    console.log(req.body)
    req.body.items.forEach(async (eachItem, itemIndex) => {
        try {
            const searchRespond = await fetch(`http://192.168.1.42:3001/api/v1/searchItem?productId=${eachItem.id}`, {
                method: 'GET',
                headers: {
                    api_key: process.env.TOKEN_SECRET
                }
            })
            const respondBody = await searchRespond.json()
            console.log(`${searchRespond.status} ${searchRespond.statusText}`, respondBody)
            let out = []
            respondBody.data.forEach(eachBin => {
                // if (eachBin.stock[0].qty >= eachItem.qty)
                out.push({ binId: eachBin.binId, mapPoint: eachBin.mapPoint, qty: eachBin.stock[0].qty })
            })
            eachItem.location = out
            if (itemIndex == req.body.items.length - 1) {
                console.log('result:', JSON.stringify(req.body.items))
                event.emit('order:add', {
                    orderId: req.body.orderId,
                    status: 'accepted',
                    items: req.body.items
                })
                //
                const routingPlan = []
            }
        }
        catch (err) {
            console.log(err.message)
        }
    })
    return res.status(200).json({
        status: 'accepted',
        message: `Order ${req.body.orderId} added to queue`
    })
}

async function getMap(req, res) {
    return res.status(200).json({
        status: 'success',
        data: showMap()
    })
}

module.exports = {
    addOrder,
    getMap
}