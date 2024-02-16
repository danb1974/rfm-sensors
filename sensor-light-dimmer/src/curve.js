const GAMMA = 2;

const MIN = 7.0e-3; //sec
const MAX = 3.0e-3; //sec
const TIMER = 2e6;  //Hz

const brightness = new Array(100).fill(0).map((_, i) => i + 1);
const ticksMin = TIMER * MIN;
const ticksMax = TIMER * MAX;

const ticksArray = brightness.map(b => {
    const power = Math.sin(b / 100.0 * Math.PI / 2);
    const corrected = Math.pow(power, GAMMA);
    const ticks = (ticksMax - ticksMin) * corrected + ticksMin;
    return Math.round(ticks);
});

console.log("// from", MIN * 1000, "to", MAX * 1000);
console.log("static const uint16_t powerToTicks[100] = {");
for (let i = 0; i < 10; i++) {
    console.log("    "
         + ticksArray.slice(i * 10, i * 10 + 10).map(n => ('     ' + n).slice(-5)).join(', ')
         + ','
    );
}
console.log("};");
