const express = require('express')
const router = express.Router()

require('dotenv').config({ path: './.env' })
const stockController = require('../controllers/stock.controller')

router
    .route('/products')
    .get(stockController.getStock)
    .delete(stockController.clearStock)

router.post('/putToLight', stockController.putToLight)

router.post('/pickToLight', stockController.pickToLight)

router.get('/products/search', stockController.searchProduct)

router.delete('/products/:productId', stockController.deleteProduct)

module.exports = router