let robots = []

const addRobot = d => {
    robots.push(new MobileRobot(d))
}

const removeRobot = d => {
    // robots.s
}

class MobileRobot {
    constructor(id, locationCode, locationCoor) {
        this.id = id
        this.locationCode = locationCode
        this.locationCoor = locationCoor
    }
    setLocation(locationCode, locationCoor) {
        this.locationCode = locationCode
        this.locationCoor = locationCoor
        return true
    }
    getLocation() {
        return {
            id: this.id,
            locationCode: this.locationCode,
            locationCoor: this.locationCoor
        }
    }
}
const onConnect = (socket, data) => {
    addRobot(data.id, data.locationCode, data.locationCoor)
}

const onInfo = (socket, data) => {
    //
}

const onMessage = (socket, data) => {
    //
}

const onPlan = (socket, data) => {
    //
}

module.exports = {
    MobileRobot,
    onConnect,
    onInfo,
    onMessage,
    onPlan
}