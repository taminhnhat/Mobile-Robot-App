function startCamera() {
    socket.emit('robot:camera:start', {})
}