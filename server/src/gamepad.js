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

const velocityGenerate = () => {
    socket.emit('ros:topic', {
        topic: 'ws_vel',
        data: {
            linear: [linear_vel_y, 0, 0],
            angular: [0, 0, angular_vel]
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
    var d = document.getElementById('centerPanel');
    d.setAttribute("id", "controller" + gamepad.index);
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
    document.body.appendChild(d);
    document.getElementById("rightPanel").remove()
    document.getElementById("leftPanel").remove()
    document.getElementById("deviceName").textContent = gamepad.id
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
    for (j in controllers) {
        var controller = controllers[j];
        var d = document.getElementById("controller" + j);
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
            document.getElementById('linear_vel').value = linear_vel_y
            document.getElementById('angular_vel').value = angular_vel
            a.innerHTML = i + ": " + controller.axes[i].toFixed(4);
            a.setAttribute("value", controller.axes[i]);
        }
    }
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