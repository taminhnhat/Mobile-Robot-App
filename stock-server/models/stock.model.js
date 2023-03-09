const mongoose = require('mongoose')

const stockSchema = new mongoose.Schema({
    binId: {
        type: Number,
        required: true
    },
    binWidth: {
        type: String,
        required: false
    },
    coordinate: {
        loca: {
            type: String,
            required: true
        },
        map: {
            type: String,
            required: true
        }
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