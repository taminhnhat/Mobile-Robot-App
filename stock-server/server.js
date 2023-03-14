const server = require('./app')
require('dotenv').config({ path: './stock-server/.env' })
const mongoose = require('mongoose')
console.log(process.env.DATABASE_URL)
mongoose.connect(process.env.DATABASE_URL, { useNewUrlParser: true })
    .catch(err => {
        console.log(err)
    })

const db = mongoose.connection
db.on('error', (error) => console.error(error))
db.once('open', () => console.log('Connected to Database'))

const port = Number(process.env.HTTP_PORT)

server.listen(port, () => {
    console.log(`HTTP Server started at port ${port}`)
})
