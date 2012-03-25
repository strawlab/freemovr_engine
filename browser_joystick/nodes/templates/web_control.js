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
    var stage = new Kinetic.Stage("container", 400, 200);
    big_radius = 200/2-4;
    button_state = [0, 0, 0, 0];

    var joyLayer = new Kinetic.Layer();
    var bgLayer = new Kinetic.Layer();
    joyCenterX = 100;
    joyCenterY = 100;

    joytip = new Kinetic.Circle({
        x: joyCenterX,
        y: joyCenterY,
        radius: 15,
        fill: "#00D2FF",
        stroke: "black",
        strokeWidth: 2,
        draggable: true
    });

    cursor_style(joytip);


    joytip.on("mouseup touchend", function() {
        joytip.x = joyCenterX;
        joytip.y = joyCenterY;
        joyLayer.draw();
        emit_joy_pos();
    });

    function emit_joy_pos() {
        // slightly funky scaling to match ROS joystick node
        axes = [-(joytip.x-joyCenterX)/big_radius,
                -(joytip.y-joyCenterY)/big_radius]

        var ms = "joystick axes: " + axes;
        $("#output").empty();
        $("#output").append(ms);
        ws.send(JSON.stringify({
            msg: "joy",
            axes: axes,
            buttons: button_state
        }));
    }

    joytip.on("dragmove", function() {
        emit_joy_pos();
    });

    joyLayer.add(joytip);

    var bg = new Kinetic.Circle({
        x: joyCenterX,
        y: joyCenterY,
        radius: big_radius,
        stroke: "black",
        strokeWidth: 2,
    });
    bgLayer.add(bg);

    stage.add(bgLayer);
    stage.add(joyLayer);

    add_buttons();

    function add_buttons() {
        var buttonLayer = new Kinetic.Layer();

        var allBtnCenterX = 300;
        var allBtnCenterY = 100;
        var btnOffset = 50;

        var cfg = { num: 0,
                    x: allBtnCenterX-btnOffset,
                    y: allBtnCenterY,
                    color: "red"
                  }
        add_button(cfg, buttonLayer);

        var cfg = { num: 1,
                    x: allBtnCenterX,
                    y: allBtnCenterY+btnOffset,
                    color: "yellow"
                  }
        add_button(cfg, buttonLayer);

        var cfg = { num: 2,
                    x: allBtnCenterX+btnOffset,
                    y: allBtnCenterY,
                    color: "green"
                  }
        add_button(cfg, buttonLayer);

        var cfg = { num: 3,
                    x: allBtnCenterX,
                    y: allBtnCenterY-btnOffset,
                    color: "blue"
                  }
        add_button(cfg, buttonLayer);

        stage.add(buttonLayer);
    }

    function add_button(cfg, buttonLayer) {
        var btnRadius = 20;
        var btn = new Kinetic.Circle({
            x: cfg.x,
            y: cfg.y,
            radius: btnRadius,
            fill: cfg.color,
            stroke: "black",
            strokeWidth: 2,
        });
        buttonLayer.add(btn);
        cursor_style(btn);

        btn.on("mousedown touchstart", function() {
            button_state[cfg.num] = 1;
            btn.fill = "black";
            buttonLayer.draw();
            emit_joy_pos();
        });

        btn.on("mouseup touchend", function() {
            button_state[cfg.num] = 0;
            btn.fill = cfg.color;
            buttonLayer.draw();
            emit_joy_pos();
        });
    }

    function cursor_style( node ) {
        // add cursor styling
        node.on("mouseover", function(){
            document.body.style.cursor = "pointer";
        });
        node.on("mouseout", function(){
            document.body.style.cursor = "default";
        });
    }
};




$(document).ready(function(){
    $("#output").empty();
    $("#output").append("Thanks for visiting!");
});
