const axios = require('axios');
const fs = require('fs')
const { generateProductList } = require('./productGenerate')
const readFromJson = fs.readFileSync('products.json')
const productList = JSON.parse(readFromJson)
console.log(productList)

function generateStock() {
    let out = []
    let filteredProductList = productList
    const count = Math.floor(Math.random() * 9 + 2)
    for (let i = 0; i < count; i++) {
        const tempIndex = Math.floor(Math.random() * (productList.length - i))
        const productId = filteredProductList[tempIndex]
        filteredProductList = filteredProductList.filter(eachProduct => {
            return eachProduct != productId
        })

        out.push({
            id: productId,
            qty: Math.floor(Math.random() * 100 + 1)
        })
    }
    return out
}

module.exports = { generateStock }
