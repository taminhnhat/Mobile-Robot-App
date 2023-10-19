const socket = io()
// console.log('script start')

document.getElementById("agentType").textContent = navigator.userAgent;
// document.getElementById(vitualjoystick).style.height = window.innerHeight - 100;

// Create JoyStick object into the DIV 'joy1Div'
var Joy1 = new JoyStick('joy1Div', { "title": "joystick1", "autoReturnToCenter": true, x: 0.4, y: 0.4 });

// var joy1IinputPosX = document.getElementById("joy1PosizioneX");
// var joy1InputPosY = document.getElementById("joy1PosizioneY");
// var joy1Direzione = document.getElementById("joy1Direzione");
var joy1X = document.getElementById("joy1X");
var joy1Y = document.getElementById("joy1Y");

// setInterval(function () { joy1IinputPosX.value = Joy1.GetPosX(); }, 50);
// setInterval(function () { joy1InputPosY.value = Joy1.GetPosY(); }, 50);
// setInterval(function () { joy1Direzione.value = Joy1.GetDir(); }, 50);
setInterval(function () { joy1X.value = Joy1.GetX(); }, 50);
setInterval(function () { joy1Y.value = Joy1.GetY(); }, 50);

// Create JoyStick object into the DIV 'joy2Div'
var Joy2 = new JoyStick('joy2Div', { "title": "joystick2", "autoReturnToCenter": true, x: 3.9, y: 3.9 });

// var joy2IinputPosX = document.getElementById("joy2PosizioneX");
// var joy2InputPosY = document.getElementById("joy2PosizioneY");
// var joy2Direzione = document.getElementById("joy2Direzione");
var joy2X = document.getElementById("joy2X");
var joy2Y = document.getElementById("joy2Y");

// setInterval(function () { joy2IinputPosX.value = Joy2.GetPosX(); }, 50);
// setInterval(function () { joy2InputPosY.value = Joy2.GetPosY(); }, 50);
// setInterval(function () { joy2Direzione.value = Joy2.GetDir(); }, 50);
setInterval(function () { joy2X.value = Joy2.GetX(); }, 50);
setInterval(function () { joy2Y.value = Joy2.GetY(); }, 50);

socket.on('ros:topic', d => {
    var item = document.createElement('li')
    item.textContent = JSON.stringify(d.data)
    var messages = document.getElementById('messages');
    messages.appendChild(item);
    var c = document.getElementById('card')
    c.scrollTo(0, c.scrollHeight)
    if (d.topic == 'ros2_state') {
        const vel = d.data.vel;
        chartData_front_right_wheel.push(vel[0]);
        while (chartData_front_right_wheel.length > chartSize) chartData_front_right_wheel.shift();
        chartData_rear_right_wheel.push(vel[1]);
        while (chartData_rear_right_wheel.length > chartSize) chartData_rear_right_wheel.shift();
        chartData_rear_left_wheel.push(vel[2]);
        while (chartData_rear_left_wheel.length > chartSize) chartData_rear_left_wheel.shift();
        chartData_front_left_wheel.push(vel[3]);
        while (chartData_front_left_wheel.length > chartSize) chartData_front_left_wheel.shift();
        // update chart
        wheelVelocityChart.data.datasets[0].data = JSON.parse(JSON.stringify(chartData_front_right_wheel));
        wheelVelocityChart.data.datasets[1].data = JSON.parse(JSON.stringify(chartData_rear_right_wheel));
        wheelVelocityChart.data.datasets[2].data = JSON.parse(JSON.stringify(chartData_rear_left_wheel));
        wheelVelocityChart.data.datasets[3].data = JSON.parse(JSON.stringify(chartData_front_left_wheel));
        wheelVelocityChart.update();
        // update bottom row
        linear_vel_average = 0;
        vel.forEach(v => linear_vel_average += v / 4)
        angular_vel_average = 0;
        vel.forEach(a => angular_vel_average += a / 4)

        document.getElementById('battery_voltage').value = d.data.bat.toFixed(2);
        linearMiniChart.data.datasets[0].data[0] = 2 + Math.abs(linear_vel_average) * 3
        linearMiniChart.data.datasets[0].data[1] = 28 - Math.abs(linear_vel_average) * 3
        linearMiniChart.update()
    }
})

let tempTable = [];
socket.on('ros:monitor', d => {
    var table = document.getElementById('nodelist')
    // remove all rows
    while (table.rows.length > 1)
        table.deleteRow(1)
    const nodes = d.nodes
    // create new rows
    let isRobotActivated = false
    nodes.forEach(node => {
        let row = table.insertRow(1)
        let cell1 = row.insertCell(0)
        let cell2 = row.insertCell(0)
        cell1.innerHTML = "---"
        cell2.innerHTML = node
        if (node == '/controller_manager')
            isRobotActivated = true
    })
    if (isRobotActivated) {
        document.getElementById('connectShortcut').style.backgroundColor = '#23D160'
        document.getElementById('connectIcon').textContent = 'link'
    }
    else {
        document.getElementById('connectShortcut').style.backgroundColor = '#f0f0f0'
        document.getElementById('connectIcon').textContent = 'link_off'
    }
})