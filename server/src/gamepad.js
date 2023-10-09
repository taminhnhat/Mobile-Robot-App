/*
 * Gamepad API Test
 * Written in 2013 by Ted Mielczarek <ted@mielczarek.org>
 *
 * To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights to this software to the public domain worldwide. This software is distributed without any warranty.
 *
 * You should have received a copy of the CC0 Public Domain Dedication along with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
 */
var haveEvents = 'GamepadEvent' in window;
var haveWebkitEvents = 'WebKitGamepadEvent' in window;
var controllers = {};
var rAF = window.mozRequestAnimationFrame ||
    window.webkitRequestAnimationFrame ||
    window.requestAnimationFrame;

var digitalTrigger = []
var analogTrigger = []
var linear_vel_x = 0
var linear_vel_y = 0
var angular_vel = 0
const joyDeadzone = 0.1

const chartSize = 20;
var linear_vel_chart = new Array(chartSize).fill(0)
var angular_vel_chart = new Array(chartSize).fill(0)
var chartData_L = new Array(chartSize).fill(0)
var chartData_A = new Array(chartSize).fill(0)

var linearVelocityChart = new Chart("linearVelocityChart", {
    type: "line",
    data: {
        labels: new Array(chartSize).fill(0),
        datasets: [{
            fill: false,
            lineTension: 0,
            backgroundColor: "rgba(0,0,255,1.0)",
            borderColor: "rgba(0,0,255,0.1)",
            data: chartData_L
        }]
    },
    options: {
        legend: { display: false },
        scales: {
            yAxes: [{ ticks: { min: -0.5, max: 0.5 } }],
        },
        overrides: {
            scales: true
        }
    }
});
var angularVelocityChart = new Chart("angularVelocityChart", {
    type: "line",
    data: {
        labels: new Array(chartSize).fill(0),
        datasets: [{
            fill: false,
            lineTension: 0,
            backgroundColor: "rgba(0,0,255,1.0)",
            borderColor: "rgba(0,0,255,0.1)",
            data: chartData_A
        }]
    },
    options: {
        legend: { display: false },
        scales: {
            yAxes: [{ ticks: { min: -2.5, max: 2.5 } }],
        }
    }
});

function updateLinearVel() {
    linear_vel_chart.push(linear_vel_y)
    while (linear_vel_chart.length > chartSize) {
        linear_vel_chart.shift()
    }
    angular_vel_chart.push(angular_vel)
    while (angular_vel_chart.length > chartSize) {
        angular_vel_chart.shift()
    }
    chartData_L = JSON.parse(JSON.stringify(linear_vel_chart))
    chartData_A = JSON.parse(JSON.stringify(angular_vel_chart))
    linearVelocityChart.data.datasets[0].data = chartData_L
    linearVelocityChart.update()
    angularVelocityChart.data.datasets[0].data = chartData_A
    angularVelocityChart.update()

}
setInterval(() => updateLinearVel(), 100)

setInterval(() => {
    document.getElementById('linear_vel').value = linear_vel_y
    document.getElementById('angular_vel').value = angular_vel
}, 100)

const velocityGenerate = () => {
    var v = (Math.abs(linear_vel_x) >= 0.1) ? linear_vel_x : 0;
    var w = (Math.abs(angular_vel) >= 0.5) ? angular_vel : 0;
    socket.emit('ros:topic', {
        topic: 'ws_vel',
        data: {
            linear: [v, 0, 0],
            angular: [0, 0, w]
        }
    })
}
const gamepadCallback = () => {
    socket.emit('gamepad', {
        buttons: digitalTrigger,
        axes: analogTrigger
    })
}

function connecthandler(e) {
    addgamepad(e.gamepad);
}

function addgamepad(gamepad) {
    controllers[gamepad.index] = gamepad;
    // var d = document.createElement("div");
    // d.setAttribute("id", "controller" + gamepad.index);
    var d = document.getElementById('Contact');
    var b = document.createElement("div");
    b.className = "buttons";
    for (var i = 0; i < gamepad.buttons.length; i++) {
        var e = document.createElement("span");
        e.className = "button";
        //e.id = "b" + i;
        e.innerHTML = i;
        b.appendChild(e);
    }
    d.appendChild(b);
    var a = document.createElement("div");
    a.className = "axes";
    for (i = 0; i < gamepad.axes.length; i++) {
        e = document.createElement("meter");
        e.className = "axis";
        //e.id = "a" + i;
        e.setAttribute("min", "-1");
        e.setAttribute("max", "1");
        e.setAttribute("value", "0");
        e.setAttribute("id", "axe" + i);
        e.innerHTML = i;
        a.appendChild(e);
    }
    d.appendChild(a);
    document.getElementById("start").style.display = "none";
    // document.body.appendChild(d);
    document.getElementById("rightPanel").style.display = "none";
    document.getElementById("leftPanel").style.display = "none";
    document.getElementById("deviceName").textContent = gamepad.id;
    rAF(updateStatus);

    setInterval(velocityGenerate, 100)
    setInterval(gamepadCallback, 100)
}

function disconnecthandler(e) {
    removegamepad(e.gamepad);
    clearInterval(velocityGenerate)
    clearInterval(gamepadCallback)
}

function removegamepad(gamepad) {
    var d = document.getElementById("controller" + gamepad.index);
    document.body.removeChild(d);
    delete controllers[gamepad.index];
}

function updateStatus() {
    scangamepads();
    digitalTrigger = []
    analogTrigger = []
    // for (j in controllers) {
    var controller = controllers[0];
    var d = document.getElementById("Contact");
    var buttons = d.getElementsByClassName("button");
    for (var i = 0; i < controller.buttons.length; i++) {
        var b = buttons[i];
        var val = controller.buttons[i];
        var pressed = val == 1.0;
        var touched = false;
        if (typeof (val) == "object") {
            pressed = val.pressed;
            if ('touched' in val) {
                touched = val.touched;
            }
            val = val.value;
        }
        digitalTrigger.push({ id: i, val: val })
        var pct = Math.round(val * 100) + "%";
        b.style.backgroundSize = pct + " " + pct;
        b.className = "button";
        if (pressed) {
            b.className += " pressed";
        }
        if (touched) {
            b.className += " touched";
        }
    }

    var axes = d.getElementsByClassName("axis");
    for (var i = 0; i < controller.axes.length; i++) {
        var a = axes[i];
        var isOutDeadZone = 1
        analogTrigger.push({ id: i, val: controller.axes[i] })
        // if (Math.abs(controller.axes[i]) > joyDeadzone) isOutDeadZone = 0
        if (i == 0) {
            linear_vel_x = Number((-0.4 * controller.axes[i] * isOutDeadZone).toFixed(2))
        }
        else if (i == 1) {
            linear_vel_y = Number((-0.4 * controller.axes[i] * isOutDeadZone).toFixed(2))
        }
        else if (i == 2) {
            angular_vel = Number((-2.0 * controller.axes[i] * isOutDeadZone).toFixed(2))
        }
        a.innerHTML = i + ": " + controller.axes[i].toFixed(4);
        a.setAttribute("value", controller.axes[i]);
    }
    // }
    rAF(updateStatus);
}

function scangamepads() {
    var gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads() : []);
    for (var i = 0; i < gamepads.length; i++) {
        if (gamepads[i] && (gamepads[i].index in controllers)) {
            controllers[gamepads[i].index] = gamepads[i];
        }
    }
}

if (haveEvents) {
    window.addEventListener("gamepadconnected", connecthandler);
    window.addEventListener("gamepaddisconnected", disconnecthandler);
} else if (haveWebkitEvents) {
    window.addEventListener("webkitgamepadconnected", connecthandler);
    window.addEventListener("webkitgamepaddisconnected", disconnecthandler);
} else {
    setInterval(scangamepads, 500);
}