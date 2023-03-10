require('dotenv').config({ path: './.env' })
const fs = require('fs')
const StockCollection = require('../models/stock.model')
const BackupCollection = require('../models/backup.model')
const { set } = require('mongoose')

let tempLightCursor = 0;
let tempBinIndex = 0;
let tempBinIndex_X = 0;
let tempBinIndex_Y = 0;

const numOfLedPerStrip = process.env.NUM_OF_LED_PER_STRIP
const numOfStrip = process.env.NUM_OF_STRIP

async function getStock(req, res) {
    const binIdFromRequest = req.query.binId || false

    if (binIdFromRequest === false) {
        try {
            const result = await StockCollection.find().sort({ mapPoint: 1 })
            const content = JSON.stringify(result)
            fs.writeFile('stock.json', content, err => {
                if (err) {
                    console.error(err);
                }
                // file written successfully
            });
            if (result == undefined)
                return res.status(500).json({
                    status: 'fail',
                    message: 'Internal Server Error'
                })
            else
                return res.status(200).json({
                    status: 'success',
                    data: result
                })
        } catch (err) {
            console.log(err)
            return res.status(500).json({
                status: 'fail',
                error: 'Internal Server Error'
            })
        }
    }
    else {
        try {
            const result = await StockCollection.find({ binId: binIdFromRequest })
            return res.status(200).json({
                status: 'success',
                data: result
            })
        }
        catch (err) {
            console.log(err)
            return res.status(500).json({
                status: 'fail',
                error: 'Internal Server Error'
            })
        }
    }
}

async function searchProduct(req, res) {
    const productIdFromRequest = req.query.productId
    const orderIdFromRequest = req.query.orderId
    const binIdFromRequest = req.query.binId
    const lightOnFlag = req.query.lightOn || 'false'
    const locationReturnFlag = req.query.locationReturn || 'false'

    // query
    let queryObj = { stocks: { $elemMatch: {} } }
    if (productIdFromRequest != undefined) queryObj.stocks.$elemMatch.productId = productIdFromRequest
    if (orderIdFromRequest != undefined) queryObj.stocks.$elemMatch.orderId = orderIdFromRequest
    if (binIdFromRequest != undefined) queryObj.binId = binIdFromRequest
    // projection
    let projectionObj = { _id: 0, coordinate: 1, binId: 1, stocks: 1 }

    try {
        // get all matched bins
        let allMatchedBin = await StockCollection.find(queryObj, projectionObj)
        // if stock is empty
        if (allMatchedBin == null) {
            return res.status(500).json({
                status: 'fail',
                message: 'Stock is empty'
            })
        }

        // filering result
        allMatchedBin.forEach(eachBin => {
            eachBin.stocks = eachBin.stocks.filter(eachProduct => {
                return eachProduct.productId == productIdFromRequest
            })
        });

        return res.status(200).json({
            status: 'success',
            data: allMatchedBin
        })
    } catch (err) {
        console.log(err)
        return res.status(500).json({
            status: 'fail',
            error: 'Internal Server Error'
        })
    }
}

async function searchBin(req, res) {
    // query
    let queryObj = { binId: req.query.binId }
    // projection
    let projectionObj = { _id: 0, binId: 1, mapPoint: 1, location: 1 }

    try {
        // get all matched bins
        let allMatchedBin = await StockCollection.find(queryObj, projectionObj)
        // if stock is empty
        if (allMatchedBin == null) {
            return res.status(500).json({
                status: 'fail',
                message: 'Stock is empty'
            })
        }

        return res.status(200).json({
            status: 'success',
            data: allMatchedBin
        })
    } catch (err) {
        console.log(err)
        return res.status(500).json({
            status: 'fail',
            error: 'Internal Server Error'
        })
    }
}

async function deleteProduct(req, res) {
    try {
        // get matched bin
        const inputProductId = String(req.params.productId)
        const allMatchedBin = await StockCollection.find({ stocks: { $elemMatch: { productId: inputProductId } } }, { _id: 0, binId: 1, stocks: 1 })
        // if stock is empty
        if (allMatchedBin == null)
            return res.status(500).json({
                status: 'fail',
                message: 'Stock is empty'
            })
        // if no bin have matched product
        if (allMatchedBin.length == 0)
            return res.status(404).json({
                status: 'fail',
                message: `Product ${inputProductId} not found`
            })
        // remove matched Product
        allMatchedBin.forEach((eachBin, index) => {
            const filteredBin = eachBin.stocks.filter(eachStock => {
                return eachStock.productId != inputProductId
            })
            eachBin.stocks = filteredBin
        })
        // return res.status(202).json(allMatchedBin)
        // update stock
        let result = []
        allMatchedBin.forEach(async (eachBin, index) => {
            const out = await eachBin.save()
            result.push(out)
            if (index == allMatchedBin.length - 1) return res.status(200).json({
                status: 'success',
                data: result
            })
        })
    } catch (err) {
        console.log(err)
        return res.status(500).json({
            status: 'fail',
            error: 'Internal Server Error'
        })
    }
}

async function createBin(req, res) {
    try {
        const stock = new StockCollection({
            binId: req.body.binId,
            mapPoint: req.body.mapPoint || 'unknown',
            location: req.body.location || 'unknown',
            stocks: []
        });
        const newStock = await stock.save()
        return res.status(201).json({
            status: 'success',
            data: newStock
        })
    } catch (err) {
        console.log(err)
        return res.status(500).json({
            status: 'fail',
            error: 'Internal Server Error'
        })
    }
}

async function putToLight(req, res) {
    // get bin with the same binId, productId, orderId
    const binList_1 = await StockCollection.find({ binId: req.body.binId, stocks: { $elemMatch: { productId: req.body.productId, orderId: req.body.orderId } } })
    try {
        if (binList_1.length == 1) {
            // update product quantity on this bin
            const thisBin = binList_1[0]
            updateProductQuantity(req, res, thisBin)
        }
        else if (binList_1.length == 0) {
            // get bin with the same binId only
            const binList_2 = await StockCollection.find({ binId: req.body.binId })
            if (binList_2.length == 1) {
                // push new product to this bin
                const thisBin = binList_2[0]
                pushNewProduct(req, res, thisBin)
            }
            else if (binList_2.length == 0) {
                // create new bin
                createNewBin(req, res)
            }
            else {
                console.log('Not expected search from db, conflict data', binList_2)
            }
        }
        else {
            console.log('Not expected search from db, conflict data', binList_1)
        }
    }
    catch (err) {
        console.log(err)
        return res.status(500).json({
            status: 'fail',
            error: 'Internal Server Error'
        })
    }
    // const allBin = await StockCollection.find({ binId: req.body.binId })
    // console.log(allBin)
    // if (allBin.length == 0) {
    //     // if no bin have the same input binId, create new bin
    //     createBin(req, res)
    // }
    // else if (allBin.length == 1) {
    //     // if one bin matched, update that bin
    //     updateBin(req, res)
    // }
    // else if (allBin.length > 1) {
    //     // bad db
    //     console.log('WARNING: SOME BINS HAVE SAME BINID', allBin)
    //     updateBin(req, res)
    // }
    // else {
    //     // log error here
    //     return res.status(500).json({
    //         status: 'fail',
    //         message: 'Stock is empty'
    //     })
    // }

    async function createNewBin(req, res) {
        //
        console.log('create new bin')

        try {
            const stock = new StockCollection({
                binId: req.body.binId,
                stocks: [{
                    productId: req.body.productId,
                    orderId: req.body.orderId,
                    productQuantity: req.body.productQuantity
                }]
            });
            const newStock = await stock.save()
            return res.status(201).json({
                status: 'success',
                data: newStock
            })
        } catch (err) {
            console.log(err)
            return res.status(500).json({
                status: 'fail',
                error: 'Internal Server Error'
            })
        }
    }

    async function updateProductQuantity(req, res, thisBin) {
        console.log('update product quantity')
        try {
            thisBin.stocks.forEach(async (eachProduct, productIndex) => {
                if (eachProduct.id == req.body.productId && eachProduct.orderId == req.body.orderId) {
                    let updateProduct = eachProduct
                    updateProduct.qty += req.body.productQuantity
                    thisBin.stocks.push(updateProduct)
                    thisBin.stocks.splice(productIndex, 1)
                    const updatedBin = await thisBin.save()
                    return res.status(201).json({
                        status: 'success',
                        data: updatedBin
                    })
                }
            })
        } catch (err) {
            console.log(err)
            return res.status(500).json({
                status: 'fail',
                error: 'Internal Server Error'
            })
        }
    }

    async function pushNewProduct(req, res, thisBin) {
        console.log('push new product')
        try {
            thisBin.stocks.push({
                id: req.body.productId,
                qty: req.body.productQuantity
            })
            const updatedBin = await thisBin.save()
            return res.status(201).json({
                status: 'success',
                data: updatedBin
            })
        } catch (err) {
            console.log(err)
            return res.status(500).json({
                status: 'fail',
                error: 'Internal Server Error'
            })
        }
    }
}

async function pickToLight(req, res) {
    try {
        // find all bin with input productId
        let allMatchedBin = await StockCollection.find({ stocks: { $elemMatch: { productId: req.body.productId } } }, { _id: 0, coordinate: 1, binId: 1, stocks: 1 })
        if (allMatchedBin.length == 0) {
            return res.status(500).json({
                status: 'fail',
                message: 'ProductId not found'
            })
        }
        else {
            allMatchedBin.forEach(eachBin => {
                // filering result
                eachBin.stocks = eachBin.stocks.filter(eachProduct => {
                    return eachProduct.productId == req.body.productId
                })
            })

            return res.status(200).json({
                status: 'success',
                data: allMatchedBin
            })
        }
    } catch (err) {
        console.log(err)
        return res.status(500).json({
            status: 'fail',
            error: 'Internal Server Error'
        })
    }
}

async function clearStock(req, res) {
    try {
        const stocks = await StockCollection.find()
        stocks.forEach(async stock => {
            await stock.remove()
        })
        res.status(200).json({
            status: 'success',
            message: 'Deleted stocks'
        })
    } catch (err) {
        console.log(err)
        res.status(500).json({
            status: 'fail',
            error: 'Internal Server Error'
        })
    }
}

async function clearBin(req, res) {
    const binId = req.query.binId || false
    if (binId === false) {

        try {
            const allBin = await StockCollection.find()
            allBin.forEach(async eachBin => {
                eachBin.stocks = []
                await eachBin.remove()
            })
            res.status(200).json({
                status: 'success',
                message: 'Deleted stocks'
            })
        } catch (err) {
            console.log(err)
            res.status(500).json({
                status: 'fail',
                error: 'Internal Server Error'
            })
        }
    }
}

async function reload(req, res) {
    try {
        const backup = await BackupCollection.find()
        if (backup.length == 0) {
            const backup = new BackupCollection({
                lightCursor: 0,
                binIndex: 0,
                binIndex_X: 0,
                binIndex_Y: 0
            })
            await backup.save()
        }
        else {
            tempLightCursor = backup[0].lightCursor
            tempBinIndex = backup[0].binIndex
            tempBinIndex_X = backup[0].binIndex_X
            tempBinIndex_Y = backup[0].binIndex_Y
        }
    } catch (err) {
        console.log(err)
    }
}

module.exports = { getStock, clearStock, createBin, putToLight, pickToLight, searchProduct, searchBin };
