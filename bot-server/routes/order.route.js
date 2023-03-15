const express = require('express')
const router = express.Router()

const orderController = require('../controllers/order.controller')

router.post('/addOrder', orderController.addOrder)
router.get('/getMap', orderController.getMap)

module.exports = router