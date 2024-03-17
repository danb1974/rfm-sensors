const initialOffset = 12345678;
const zeroPulses = [
    initialOffset + 19600,
    initialOffset + 40100,
    initialOffset + 40600,//
    initialOffset + 55000,//
    initialOffset + 59900,
    //
    initialOffset + 95000,//
    initialOffset + 100001,
    initialOffset + 100002,//
    initialOffset + 120000,
    initialOffset + 139900,
    initialOffset + 165000,//
];

function validPulse(pulse) {
    let hits = 0;
    for (let i = 0; i < zeroPulses.length; i++) {
        let zeroPulse = zeroPulses[i];
        let offset = (Math.abs(pulse - zeroPulse)) % 20000;
        if (offset >= 10000) {
            offset = 20000 - offset;
        }
        if (offset < 500) {
            hits++;
        }
    }
    //console.log(hits);
    return hits > 5;
}

let oldPulse = -100000;
for (i = 0; i < zeroPulses.length; i++) {
    let pulse = zeroPulses[i];
    
    if (pulse - oldPulse < 19500) {
        console.log("Skipping early pulse", pulse);
        continue;
    }

    oldPulse = pulse;
    let result = validPulse(pulse);
    console.log("Testing pulse", pulse, "result", result);
}
