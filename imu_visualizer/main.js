var express = require('express');
var app = express();
var serv = require('http').Server(app);
const PORT = 2000;
const PACKET_INTERVAL = 50; //The time, in milliseconds between each data packet being sent

var date = new Date();
const START_TIME = date.getTime()

app.get('/',function(req, res) {
    res.sendFile(__dirname + '/client/index.html');
});
app.use('/client',express.static(__dirname + '/client'));

serv.listen(PORT);
console.log("Visualizer hosted on port " + PORT); //If you see this the website should be up

SOCKET_LIST = {};

var IMUData = function(accel, vel, pos, magRaw, rotVelocity, rotation){
    d = new Date();
    var self = {
        accel:accel,
        vel:vel,
        pos:pos,
        magRaw:magRaw,
        rotVelocity:rotVelocity,
        rotation:rotation,
        time:d.getTime()-START_TIME,
    };
    return self;
}

var io = require('socket.io')(serv,{});

io.sockets.on('connection', function(socket){
    socket.id = Math.random();
    console.log("Socket " + socket.id + " connected");
    SOCKET_LIST[socket.id] = socket;
  
    socket.on('disconnect',function(){
      console.log("Socket " + socket.id + " disconnected");
      delete SOCKET_LIST[socket.id];
    });
});

setInterval(function(){
    packet = new IMUData([0,0,1],[0,0,0],[0,0,0],[1,0,0],[0,1,1],[0,0,1]);
    //we just need to get our data values in here!! just add a c++ addon uwu
    //console.log(packet);
    for(socket in SOCKET_LIST){
        SOCKET_LIST[socket].emit('IMUData', packet);
    }
},PACKET_INTERVAL);