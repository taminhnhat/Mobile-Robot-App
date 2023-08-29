const socket = io()
console.log('script start')
// Create JoyStick object into the DIV 'joy1Div'
var Joy1 = new JoyStick('joy1Div', { x: 0.4, y: 0.4 });

var joy1IinputPosX = document.getElementById("joy1PosizioneX");
var joy1InputPosY = document.getElementById("joy1PosizioneY");
var joy1Direzione = document.getElementById("joy1Direzione");
var joy1X = document.getElementById("joy1X");
var joy1Y = document.getElementById("joy1Y");

setInterval(function () { joy1IinputPosX.value = Joy1.GetPosX(); }, 50);
setInterval(function () { joy1InputPosY.value = Joy1.GetPosY(); }, 50);
setInterval(function () { joy1Direzione.value = Joy1.GetDir(); }, 50);
setInterval(function () { joy1X.value = Joy1.GetX(); }, 50);
setInterval(function () { joy1Y.value = Joy1.GetY(); }, 50);

// Create JoyStick object into the DIV 'joy2Div'
var joy2Param = { "title": "joystick2", "autoReturnToCenter": true, x: 3.9, y: 3.9 };
var Joy2 = new JoyStick('joy2Div', joy2Param);

var joy2IinputPosX = document.getElementById("joy2PosizioneX");
var joy2InputPosY = document.getElementById("joy2PosizioneY");
var joy2Direzione = document.getElementById("joy2Direzione");
var joy2X = document.getElementById("joy2X");
var joy2Y = document.getElementById("joy2Y");

setInterval(function () { joy2IinputPosX.value = Joy2.GetPosX(); }, 50);
setInterval(function () { joy2InputPosY.value = Joy2.GetPosY(); }, 50);
setInterval(function () { joy2Direzione.value = Joy2.GetDir(); }, 50);
setInterval(function () { joy2X.value = Joy2.GetX(); }, 50);
setInterval(function () { joy2Y.value = Joy2.GetY(); }, 50);

socket.on('ros:topic', d => {
    var item = document.createElement('li')
    item.textContent = JSON.stringify(d)
    var messages = document.getElementById('messages');
    messages.appendChild(item);
    var c = document.getElementById('card')
    c.scrollTo(0, c.scrollHeight)
})