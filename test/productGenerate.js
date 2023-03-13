const fs = require('fs')

function generateProductList(quantum) {
    let out = []
    for (let i = 1; i <= quantum; i++) {
        let newProductId = `${i}`
        for (let k = newProductId.length; k < 4; k++) {
            newProductId = '0' + newProductId
        }
        for (let j = 1; j <= 9; j++) {
            newProductId = Math.floor(Math.random() * 10) + newProductId
        }
        out.push(newProductId)
    }
    const data = JSON.stringify(out)
    fs.writeFileSync('products.json', data)
    return out
}
generateProductList(900)

module.exports = { generateProductList }