const socket = io('/')
const myPeer = new Peer(undefined, {
    host: '/',
    port: '3001'
})
const videoGrid = document.getElementById('video-grid')
const myVideo = document.createElement('video')
myVideo.muted = true

navigator.mediaDevices.getUserMedia({
    video: true,
    audio: true
}).then(stream => {
    addVideoStream(myVideo, stream)

    socket.on('user-connected', userId => {
        connectToUser(userId, stream)
    })
})

myPeer.on('open', id => {
    socket.emit('join-room', ROOM_ID, id)
})

function connectToUser(userId, stream) {
    const call = myPeer.call(userId, stream)
    const video = document.createElement('video')
    // when the other user call back
    call.on('stream', userVideoStream => {
        addVideoStream(video, userVideoStream)
    })
    call.on('close', () => {
        video.remove()
    })
}

function addVideoStream(video, stream) {
    video.srcObject = stream
    video.addEventListener('loadedmetadata', () => {
        video.play()
    })
    document.getElementById('video-grid').appendChild(video)
}