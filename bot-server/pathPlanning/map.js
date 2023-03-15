const MobileRobot = require('./bot')
let whMap = []
let groundLayer = []
let guidanceLayer = []
let statisObstacleLayer = []
// create base
for (let i = 1; i <= 50; i++) {
    whMap.push(createEmptyCol(30))
    groundLayer.push(createEmptyCol(30))
    guidanceLayer.push(createEmptyCol(30))
    statisObstacleLayer.push(createEmptyCol(30))
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

function createEmptyCol(num) {
    let row = []
    for (let i = 1; i <= num; i++) {
        row.push({ value: 0, type: 'empty ground', colorCode: 'rgb(200,200,200)' })
    }
    return row
}

function createGuidance(startPoint, endPoint) {
    for (let i = startPoint.x; i <= endPoint.x; i++) {
        for (let j = startPoint.y; j <= endPoint.y; j++) {
            whMap[j - 1][i - 1] = { value: 0, type: 'global guidance', colorCode: 'rgb(100,100,100)' }
            guidanceLayer[j - 1][i - 1] = { value: 0, type: 'global guidance', colorCode: 'rgb(100,100,100)' }
        }
    }
}

function createStaticObstacle(startPoint, endPoint) {
    for (let i = startPoint.x; i <= endPoint.x; i++) {
        for (let j = startPoint.y; j <= endPoint.y; j++) {
            whMap[j - 1][i - 1] = { value: 1, type: 'static obstacle', colorCode: 'rgb(0,95,50)' }
            statisObstacleLayer[j - 1][i - 1] = { value: 1, type: 'static obstacle', colorCode: 'rgb(0,95,50)' }
        }
    }
}

function showMap() {
    return {
        fullMap: whMap,
        groundMap: groundLayer,
        guidanceMap: guidanceLayer,
        statisObstacleMap: statisObstacleLayer
    }
}

module.exports = { showMap }