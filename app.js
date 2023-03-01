// express module
const express = require('express')
const app = express()
app.use(express.json())

const http = require("http")
const fs = require("fs")

// env
require('dotenv').config()

// logger
const morgan = require('morgan')
morgan.token('body', req => {
    return JSON.stringify(req.body)
})
if (process.env.NODE_ENV == 'production')
    app.use(morgan(':remote-addr - :remote-user [:date[clf]] ":method :url HTTP/:http-version" :status :body :res[content-length] :response-time ms'))
else if (process.env.NODE_ENV == 'development')
    app.use(morgan(':remote-addr - :remote-user [:date[clf]] ":method :url HTTP/:http-version" :status :response-time ms'))
else
    app.use(morgan('common'))

// // authorization
// const auth = require('./middlewares/auth')
// app.use(auth)

// routing
const orderRouter = require('./routes/order.routes')
app.use('/api/v1', orderRouter)

// create server
const httpServer = http.createServer(app)

// export
module.exports = httpServer