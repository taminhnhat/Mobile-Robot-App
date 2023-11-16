document.getElementById('startCameraButton').addEventListener('click', () => {
    socket.emit('robot:camera:start', {})
})
document.getElementById('stopCameraButton').addEventListener('click', () => {
    socket.emit('robot:camera:stop', {})
})