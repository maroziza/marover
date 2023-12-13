
var gateway = `ws://${window.location.hostname}/ws`;

var websocket;
// Init web socket when the page loads
window.addEventListener('load', onload);

// function createStream(){
//    var streamSource = `http://${window.location.hostname}/stream`;
//     var elem = document.createElement("img");
//     elem.setAttribute("src", streamSource);
//     document.getElementById("stream").appendChild(elem);
// }

function onload(event) {
    initWebSocket();
    //postpone stream creation after ws initialization
    // createStream();
    // sendAxisWs([0, 0]);
}

function sendAxisWs(axisData){
    websocket.send(JSON.stringify({
        type: "axis",
        axis: axisData,
        min: -100,
        max: 100
    }) 
    );
}

function sendLedWs(){
    websocket.send(JSON.stringify({
        type: "led"
    }) 
    );
}



function initWebSocket() {
    console.log('Trying to open a WebSocket connection…');
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
}

// When websocket is established, call the getReadings() function
function onOpen(event) {
    console.log('Connection opened');
    // getReadings();
}

function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
}

// Function that receives the message from the ESP32 with the readings
function onMessage(event) {
    console.log(event.data);
    // var myObj = JSON.parse(event.data);
    // var keys = Object.keys(myObj);

    // for (var i = 0; i < keys.length; i++){
    //     var key = keys[i];
    //     document.getElementById(key).innerHTML = myObj[key];
    // }
}