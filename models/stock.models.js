const mongoose = require('mongoose')

const stockSchema = new mongoose.Schema({
    binId: {
        type: String,
        required: true
    },
    coordinate: {
        shelfId: {
            type: String,
            required: true
        },
        binId: {
            type: Number,
            required: true
        },
        X_index: {
            type: Number,
            required: true
        },
        Y_index: {
            type: Number,
            required: true
        }
    },
    stocks: {
        type: Array,
        required: true,
        default: []
    },
    dateCreated: {
        type: Date,
        required: true,
        default: Date.now
    }
})

module.exports = mongoose.model('Stock', stockSchema)