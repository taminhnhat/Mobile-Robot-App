const express = require('express')
const router = express.Router()

router.post('/addOrder', (req, res) => {
    console.log(req.body)
    res.status(200).json({
        status: 'success',
        message: 'order added'
    })
})

module.exports = router