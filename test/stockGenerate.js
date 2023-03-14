const axios = require('axios');
const { generateKey } = require('crypto');
const fs = require('fs')
const readFromJson = fs.readFileSync('products.json')
const productList = JSON.parse(readFromJson)

function generateStock() {
    let out = []
    let filteredProductList = productList
    const count = Math.floor(Math.random() * 9 + 2)
    for (let i = 0; i < count; i++) {
        const tempIndex = Math.floor(Math.random() * (productList.length - i))
        const productId = filteredProductList[tempIndex].id
        const productBarcode = filteredProductList[tempIndex].barcode
        filteredProductList = filteredProductList.filter(eachProduct => {
            return eachProduct.id != productId
        })

        out.push({
            id: productId,
            barcode: productBarcode,
            qty: Math.floor(Math.random() * 100 + 1)
        })
    }
    return out
}
console.log(generateStock())

module.exports = { generateStock }
