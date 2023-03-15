class MobileRobot {
    constructor(id, loc) {
        this.id = id
        this.locationCode = loc
        this.locationCoor = loc
    }
    setLocation(locationCode, locationCoor) {
        this.locationCode = locationCode
        this.locationCoor = locationCoor
    }
}

module.exports = MobileRobot