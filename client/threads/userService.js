const { parentPort, workerData } = require('worker_threads');
const { exec, execSync } = require('child_process');

let services = {
    murin_bringup: { loaded: 'unknown', active: 'unknown' },
    murin_server: { loaded: 'unknown', active: 'unknown' },
    murin_client: { loaded: 'unknown', active: 'unknown' },
    murin_websocket: { loaded: 'unknown', active: 'unknown' },
}
services = []
let count = 0

parentPort.on('error', err => console.log(err));
parentPort.on('message', msg => {
    if (msg == 'run') isRefreshed = false
});

const debug = false

const process = () => {
    count++
    if (debug) console.log('service thread: ', count)
    services = []
    // get murin_bringup status
    const getMurinBringupStatus = () => {
        let _Loaded = 'unknown'
        let _Active = 'unknown'
        try {
            const out = execSync('systemctl --user status murin_bringup.service')
            const d = String(out).split('\n')
            _Loaded = d[1].trim().split(':')[1].split(' ')[1]
            _Active = d[2].trim().split(':')[1].split(' ')[1]
            // console.log('murin_bringup.service', _Loaded, _Active)
        }
        catch (err) { }
        services.push({
            service: 'murin_bringup',
            loaded: _Loaded,
            active: _Active
        })
    }
    getMurinBringupStatus()
    // get murin_server status
    const getMurinServerStatus = () => {
        let _Loaded = 'unknown'
        let _Active = 'unknown'
        try {
            const out = execSync('systemctl --user status murin_server.service')
            const d = String(out).split('\n')
            _Loaded = d[1].trim().split(':')[1].split(' ')[1]
            _Active = d[2].trim().split(':')[1].split(' ')[1]
            // console.log('murin_server.service', _Loaded, _Active)
        }
        catch (err) { }
        services.push({
            service: 'murin_server',
            loaded: _Loaded,
            active: _Active
        })
    }
    getMurinServerStatus()
    // get murin_client status
    const getMurinClientStatus = () => {
        let _Loaded = 'unknown'
        let _Active = 'unknown'
        try {
            const out = execSync('systemctl --user status murin_client.service')
            console.log(String(out))
            const d = String(out).split('\n')
            _Loaded = d[1].trim().split(':')[1].split(' ')[1]
            _Active = d[2].trim().split(':')[1].split(' ')[1]
            // console.log('murin_client.service', _Loaded, _Active)
        }
        catch (err) { }
        services.push({
            service: 'murin_client',
            loaded: _Loaded,
            active: _Active
        })
    }
    getMurinClientStatus()
    parentPort.postMessage(services)

    setTimeout(process, 3000)
}

process()