const mongoose = require('mongoose')

const stockSchema = new mongoose.Schema({
    binId: {
        type: String,
        required: true
    },
    mapPoint: {
        type: String,
        required: true
    },
    location: {
        type: String,
        required: true
    },
    stocks: {
        type: Array,
        required: true,
        default: []
    },
    lastUpdate: {
        type: Date,
        required: true,
        default: Date.now
    }
})

module.exports = mongoose.model('Stock', stockSchema)