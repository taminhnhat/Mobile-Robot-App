const express = require('express')
const router = express.Router()

require('dotenv').config({ path: './.env' })
const stockController = require('../controllers/stock.controller')

router.get('/getStock', stockController.getStock)
router.post('/createBin', stockController.createBin)
router.delete('/clearStock', stockController.clearStock)
router.delete('/clearBin', stockController.clearBin)

router.post('/putToLight', stockController.putToLight)

router.post('/pickToLight', stockController.pickToLight)

router.get('/searchItem', stockController.searchProduct)
router.get('/searchBin', stockController.searchBin)

module.exports = router