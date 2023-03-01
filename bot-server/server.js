const app = require('express')();
const http = require('http').Server(app);
const io = require('socket.io')(http);
const port = process.env.PORT || 3000;

const botController = require('.')

app.get('/', (req, res) => {
    res.sendFile(__dirname + '/index.html');
});

const {
    createTodo,
    readTodo,
    updateTodo,
    deleteTodo,
    listTodo,
} = createTodoHandlers(components);

io.on('connection', (socket) => {
    socket.on('chat message', msg => {
        io.emit('chat message', msg);
    });
    socket.on("picking:newPlan", createTodo);
    socket.on("picking:check", readTodo);

    socket.on("putting:newPlan", updateTodo);
    socket.on("putting:check", deleteTodo);

    socket.on("bot:complete", listTodo);
    socket.on("bot:status", handleBotStatus);
});

http.listen(port, () => {
    console.log(`Socket.IO server running at http://localhost:${port}/`);
});


function handleBotStatus() {
    //
}