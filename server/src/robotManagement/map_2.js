const MobileRobot = require('../robots')
require('dotenv').config({ path: './bot-server/.env' })
let scaleFactor = Math.floor(1000 / Number(process.env.MAP_RESOLUSION))
if (scaleFactor <= 0) scaleFactor = 1
else if (scaleFactor > 4) scaleFactor = 4
let whMap = []
let groundLayer = []
let guidanceLayer = []
let statisObstacleLayer = []
// create base
for (let i = 1; i <= scaleFactor * 50; i++) {
    whMap.push(createEmptyCol(scaleFactor * 30))
    groundLayer.push(createEmptyCol(scaleFactor * 30))
    guidanceLayer.push(createEmptyCol(scaleFactor * 30))
    statisObstacleLayer.push(createEmptyCol(scaleFactor * 30))
}
// create guidance
for (let i = 0; i <= 12; i++) {
    createGuidance({ x: 3, y: 4 * i + 2 }, { x: 14, y: 4 * i + 2 })
    createGuidance({ x: 17, y: 4 * i + 2 }, { x: 28, y: 4 * i + 2 })
}
createGuidance({ x: 15, y: 1 }, { x: 16, y: 50 })
createGuidance({ x: 2, y: 2 }, { x: 2, y: 50 })
createGuidance({ x: 29, y: 2 }, { x: 29, y: 50 })

for (let i = 1; i <= 12; i++) {
    createStaticObstacle({ x: 4, y: 4 * i }, { x: 12, y: 4 * i })
    createStaticObstacle({ x: 19, y: 4 * i }, { x: 27, y: 4 * i })
}

console.log(whMap.length, typeof (whMap.length))

function createEmptyCol(num) {
    let row = []
    for (let i = 1; i <= num; i++) {
        row.push({ value: 0, type: 'empty ground', colorCode: 'rgb(200,200,200)' })
    }
    return row
}

function createGuidance(startPoint, endPoint) {
    for (let i = scaleFactor * startPoint.x - scaleFactor + 1; i <= scaleFactor * endPoint.x; i++) {
        for (let j = scaleFactor * startPoint.y - scaleFactor + 1; j <= scaleFactor * endPoint.y; j++) {
            whMap[j - 1][i - 1] = { value: 0, type: 'global guidance', colorCode: 'rgb(100,100,100)' }
            guidanceLayer[j - 1][i - 1] = { value: 1, type: 'global guidance', colorCode: 'rgb(100,100,100)' }
        }
    }
}

function createStaticObstacle(startPoint, endPoint) {
    for (let i = scaleFactor * startPoint.x - scaleFactor + 1; i <= scaleFactor * endPoint.x; i++) {
        for (let j = scaleFactor * startPoint.y - scaleFactor + 1; j <= scaleFactor * endPoint.y; j++) {
            whMap[j - 1][i - 1] = { value: 1, type: 'static obstacle', colorCode: 'rgb(0,95,50)' }
            statisObstacleLayer[j - 1][i - 1] = { value: 2, type: 'static obstacle', colorCode: 'rgb(0,95,50)' }
        }
    }
}

function heatmap(data) {
    let res = []
    return res
}

function showMap() {
    return {
        fullMap: whMap,
        groundMap: groundLayer,
        guidanceMap: guidanceLayer,
        statisObstacleMap: statisObstacleLayer
    }
}

function robotPlot(data) {
    console.log(data)
}

module.exports = { showMap, robotPlot }