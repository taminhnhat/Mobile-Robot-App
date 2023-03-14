const fs = require('fs')
const orderCollection = require('../models/order.model')

async function addOrder(req, res) {
    console.log(req.body)
    return res.status(200).json({
        status: 'accepted',
        message: `Order ${req.body.orderId} added to queue`
    })
}

module.exports = { addOrder }