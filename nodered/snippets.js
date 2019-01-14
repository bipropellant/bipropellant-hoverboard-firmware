//////////////////////////////////////////////////////////////////////////////////////////////
// input function, takes a buffer in msg.payload;
// parses the protocol, and generates ACK, etc.
//                             node.send({payload: s.curr_msg.bytes.slice(0, s.curr_msg.len)});
// sends data (cmd|data only) for processing
//    node.send([null, {payload: b}]);
// sends direct to serial port

var s = flow.get('s') || {
    state: 0,
    count: 0,
    CS: 0,
    curr_msg:{
        SOM:2,
        CI:0,
        len:0,
        bytes: new Buffer(256)
    },
    curr_msg_bytes: null,
    ascii: '',
    CI: 0,

};

flow.set('s', s);

var states = {
    PROTOCOL_STATE_IDLE:0,
    PROTOCOL_STATE_WAIT_CI:1,
    PROTOCOL_STATE_WAIT_LEN:2,
    PROTOCOL_STATE_WAIT_END:3,
};


var PROTOCOL_SOM = 2;

var send_ack = function(CI) {
    var b = new Buffer(5);
    b[0] = 2;
    b[1] = CI;
    b[2] = 1;
    b[3] = 'A'.charCodeAt(0);
    b[4] = (0 - b[1] - b[2] - b[3]) & 0xff;
    
    node.send([null, {payload: b}]);
}


var protocol_byte = function( byte ){
    //node.warn(byte + " "+ s.state);
    switch(s.state){
        case states.PROTOCOL_STATE_IDLE:
            if (byte == PROTOCOL_SOM){
                s.curr_msg.SOM = byte;
                s.state = states.PROTOCOL_STATE_WAIT_CI;
                s.CS = 0;
            } else {
                //////////////////////////////////////////////////////
                // if the byte was NOT SOM (02), then treat it as an 
                // ascii protocol byte.  BOTH protocol can co-exist
                //ascii_byte( byte );
                //////////////////////////////////////////////////////
                if (byte >= 0x20) {
                    s.ascii += String.fromCharCode(byte);
                }
                if (byte == 10) {
                    if (s.ascii){
                        node.send([null, null, {payload:s.ascii}]);
                    }
                    s.ascii = '';
                }

                if (byte == 13) {
                    if (s.ascii){
                        node.send([null, null, {payload:s.ascii}]);
                    }
                    s.ascii = '';
                }
                
            }
            break;
        case states.PROTOCOL_STATE_WAIT_CI:
            s.curr_msg.CI = byte;
            s.CS += byte;
            s.state = states.PROTOCOL_STATE_WAIT_LEN;
            break;
        case states.PROTOCOL_STATE_WAIT_LEN:
            s.curr_msg.len = byte;
            s.curr_msg_bytes = new Buffer(s.curr_msg.len+4);
            s.curr_msg_bytes[0] = 2;
            s.curr_msg_bytes[1] = s.curr_msg.CI;
            s.curr_msg_bytes[2] = s.curr_msg.len;
            
            s.count = 0;
            s.CS += byte;
            s.state = states.PROTOCOL_STATE_WAIT_END;
            break;
        case states.PROTOCOL_STATE_WAIT_END:
            if (!s.curr_msg_bytes){
                s.state = states.PROTOCOL_STATE_IDLE;
                break;
            }
            s.curr_msg_bytes[s.count+3] = byte;
            s.curr_msg.bytes[s.count++] = byte;
            s.CS += byte;
            s.CS &= 0xff;
            if (s.count >= s.curr_msg.len+1){
                if (s.CS !== 0){
                    node.warn('BAD CS');
                    var m = [];
                    m.push(s.curr_msg.SOM);
                    m.push(s.curr_msg.CI);
                    m.push(s.curr_msg.len);
                    for (var x = 0; x < s.curr_msg.len; x++){
                        m.push(s.curr_msg.bytes[x]);
                    }
                    m.push(s.CS);
                    node.warn('bad cs'+ util.inspect(m)+"xxx"+ util.inspect(s.curr_msg_bytes));
                    var msgs = flow.get("msgs");
                    //node.warn(msgs);
                    
                    if (msgs && msgs[s.curr_msg.bytes[1]]){
                        node.warn("last msg", msgs[s.curr_msg.bytes[1]]);
                    }
                } else {
                    switch (s.curr_msg.bytes[0]){
                        case 'N'.charCodeAt(0):
                            node.warn('NACK');
                            break;
                        case 'A'.charCodeAt(0):
                            //node.warn('ACK');
                            break;
                        default:
                            node.send({payload: s.curr_msg.bytes.slice(0, s.curr_msg.len)});
                            send_ack(s.curr_msg.CI);
                            break;
                    }
                    //node.warn(s.curr_msg);
                }
                //node.warn("set idle");
                s.state = states.PROTOCOL_STATE_IDLE;
            }
            break;
    }
};

for (var i = 0; i < msg.payload.length; i++){
    protocol_byte(msg.payload[i]);
}

// end of input function, takes a buffer in msg.payload;
//////////////////////////////////////////////////////////////////////////////////////////////





//////////////////////////////////////////////////////////////////////////////////////////////
// parse 'node.send(msg)' from above
//
// node.send([null, msg]) - sends a message to send back to the HB
//
var out = '';
switch (msg.payload[0]){
    case 't'.charCodeAt(0):
        out = {test:'test return len'+msg.payload.length};
        break;
    case 'R'.charCodeAt(0):
        switch(msg.payload[1]) {
            case 1:
                out = {
                    sensordata:{
                        Angle:[msg.payload.readInt16LE(2+0),
                            msg.payload.readInt16LE(2+28+0)],
                        Roll:[msg.payload.readInt16LE(2+7+0),
                            msg.payload.readInt16LE(2+28+7+0)],
                        raw:msg.payload.slice(2),
                        
                    }
                };
                break;
            case 4:
                out = {
                    hallmm:{
                        L:msg.payload.readInt32LE(2),
                        R:msg.payload.readInt32LE(6),
                        LO:msg.payload.readInt32LE(10),
                        RO:msg.payload.readInt32LE(14),
                    }
                };
                // reset the offsets
                msg.payload[0] = 'W'.charCodeAt(0);
                node.send([null, msg]);
                break;
        }
        break;
        
}

if (out) {
    msg.payload = out;
    return msg;
}

//
//////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////
// create a message from a buffer.
// buffer = cmd|data
// output to serial port.

var len = msg.payload.length;
var test = new Buffer(len+4);

var CI = flow.get("CI") || 0;
CI = ((CI+1)%256);
flow.set('CI', CI);

test[0] = 0x02;
test[1] = CI;
test[2] = len;


// one less, because cmd is included
var cs = 0;

var i;
for (i = 1; i < test.length-1; i++){
    if (i > 2) {
        test[i] = msg.payload[i-3];
    }
    cs -= test[i];
}
test[i] = cs & 0xff;

var msgs = flow.get("msgs") || {};
msgs[CI] = test;
flow.set("msgs", msgs);

msg.payload = test;

return msg;
//////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////
// create a hall request msg.
var len = 2;
var test = new Buffer(len);

test[0] = "R".charCodeAt(0);
test[1] = 4;

msg.payload = test;

return msg;
//////////////////////////////////////////////////////////////




//////////////////////////////////////////////////////////////
// create a test msg.
var len = ((Math.random()*30)>>0)+1;
var test = new Buffer(len);

test[0] = "T".charCodeAt(0);

var start = (Math.random()*256)>>0;

for (i = 1; i < len; i++){
    test[i] = (start+i) & 0xff;
}

msg.payload = test;

return msg;
//////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////
// create a msg to stop debug and turn off poweroff.
// sent as ascii direct to serial port.....
var out = new Buffer(4);
out[0] = "E".charCodeAt(0);
out[1] = "\n".charCodeAt(0);
out[2] = "P".charCodeAt(0);
out[3] = "\n".charCodeAt(0);

msg.payload = out;

flow.set('s',  null);

return msg;
//////////////////////////////////////////////////////////////
