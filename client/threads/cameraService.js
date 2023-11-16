const { parentPort, workerData } = require('worker_threads');
const { exec, execSync } = require('child_process');

parentPort.on('error', err => console.log(err));
parentPort.on('message', msg => {
    try {
        execSync('source /home/nhattm/dev-ws/install/setup.bash')
        execSync('ros2 launch murin_bringup realsense.launch.py')
    }
    catch (err) {
        console.log(err)
    }
});

const process = () => {
}

process()