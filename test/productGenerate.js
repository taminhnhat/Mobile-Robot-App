const fs = require('fs')

function generateProductList(quantum) {
    let out = []
    for (let i = 1; i <= quantum; i++) {
        let newBarcode = ''
        let newProductId = `${i}`
        for (let k = newProductId.length; k < 5; k++) {
            newProductId = '0' + newProductId
        }
        for (let j = 1; j <= 13; j++) {
            newBarcode += Math.floor(Math.random() * 10)
        }
        out.push({ id: newProductId, barcode: newBarcode })
    }
    const data = JSON.stringify(out)
    fs.writeFileSync('products.json', data)
    return out
}
generateProductList(900)

module.exports = { generateProductList }