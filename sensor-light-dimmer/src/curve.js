const TIMER = 2e6;  //Hz

function genDelayMap(min, max, algo, comment) {
    const brightness = new Array(100).fill(0).map((_, i) => i + 1);
    const ticksMin = TIMER * min;
    const ticksMax = TIMER * max;
    
    const ticksArray = brightness.map(b => {
        switch (algo) {
            case 1:
                const power1 = Math.sin(b / 100.0 * Math.PI / 2);
                const corrected1 = Math.pow(power1, 2);
                const ticks1 = (ticksMax - ticksMin) * corrected1 + ticksMin;
                return Math.round(ticks1);
                break;
            case 2:
                const power2 = Math.sin(b / 100.0 * Math.PI / 2);
                const ticks2 = (ticksMax - ticksMin) * power2 + ticksMin;
                return Math.round(ticks2);
                break;
            case 3:
                const ticks3 = (ticksMax - ticksMin) * b / 100.0 + ticksMin;
                return Math.round(ticks3);
                break;
        }
    });

    console.log("//", comment);
    console.log("// from", min * 1000, "to", max * 1000, "algo", algo);
    console.log("static const uint16_t powerToTicks[100] = {");
    for (let i = 0; i < 10; i++) {
        console.log("    "
             + ticksArray.slice(i * 10, i * 10 + 10).map(n => ('     ' + n).slice(-5)).join(', ')
             + ','
        );
    }
    console.log("};");
    console.log();
}

genDelayMap(7.0e-3, 3.5e-3, 1, "Philips LedLustre, sinus + gamma curve");
genDelayMap(6.2e-3, 3.5e-3, 3, "Generic modern led, linear curve");

genDelayMap(7.0e-3, 3.8e-3, 1, "Philips LedLustre, sinus + gamma curve");
genDelayMap(6.2e-3, 3.8e-3, 3, "Generic modern led, linear curve");

genDelayMap(7.0e-3, 3.0e-3, 1, "Philips LedLustre, sinus + gamma curve");
genDelayMap(6.2e-3, 3.0e-3, 3, "Generic modern led, linear curve");
