var ws_uri = "ws://{{ base_url }}/{{ echo_ws_path }}";

function lag()
{
    var start = (new Date).getTime();
    ws.send(JSON.stringify({
        msg: "lag",
        start: start
    }));
}

function init()
{
    // global var
    ws = new WebSocket(ws_uri);

    ws.onopen = function() {
    };
    ws.onmessage = function (evt) {
        var msg = jQuery.parseJSON(evt.data);

        var start = msg.start;
        var stop = (new Date).getTime();
        var diff = stop - start;
        $("#output").empty();
        $("#output").append("lag: "+ diff + " msec" );
    };
}

window.addEventListener("load", init, false);

window.onload = function() {
    var stage = new Kinetic.Stage("container", 200, 200);
    var joyLayer = new Kinetic.Layer();
    var bgLayer = new Kinetic.Layer();
    rectX = stage.width / 2;
    rectY = stage.height / 2;

    joytip = new Kinetic.Circle({
        x: rectX,
        y: rectY,
        radius: 15,
        fill: "#00D2FF",
        stroke: "black",
        strokeWidth: 2,
        draggable: true
    });

    // add cursor styling
    joytip.on("mouseover", function(){
        document.body.style.cursor = "pointer";
    });
    joytip.on("mouseout", function(){
        document.body.style.cursor = "default";
    });

    joytip.on("mouseup touchend", function() {
        joytip.x = rectX;
        joytip.y = rectY;
        joyLayer.draw();
        emit_joy_pos();
    });

    function emit_joy_pos() {
        var ms = "mouse: " + joytip.x + ", " + joytip.y;
        $("#output").empty();
        $("#output").append(ms);
        ws.send(JSON.stringify({
            msg: "mouse",
            x: (joytip.x-rectX),
            y: (joytip.y-rectY)
        }));
    }

    joytip.on("dragmove", function() {
        emit_joy_pos();
    });

    joyLayer.add(joytip);

    var radius = Math.min(stage.width, stage.height)/2 - 4;
    var bg = new Kinetic.Circle({
        x: rectX,
        y: rectY,
        radius: radius,
        stroke: "black",
        strokeWidth: 2,
    });
    bgLayer.add(bg);

    stage.add(bgLayer);
    stage.add(joyLayer);
};




$(document).ready(function(){
    $("#output").empty();
    $("#output").append("Thanks for visiting!");
});
