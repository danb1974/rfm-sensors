#### sensor-door

##### outgoing

```
'B', b, 'S', s
b = voltage / 10 - 100
s = !input or 0xFF
```

---

#### sensor-light-dimmer

##### outgoing

```
0x01 (init)
```

```
0x02, b (send state)
b = brightness
```

##### incoming

```
0x01, b (set brightness)
b = brightness
```

```
0x02 (send state)
```

```
0x03, m, b (set mode)
m = mode
b = modeNoDimmerBrightness
```

```
0x04, b (set led brighness)
b = ledbrightness
```

```
0x05, b, r (activate min brightness)
b = minBrightness
r = minBrightnessReset (seconds)
```

---

#### sensor-presence

##### outgoing

```
'B', b, 'L', ll, 'S', s
b = voltage / 10 - 100
ll = light (MSB LSB)
s = input
```

---

##### sensor-thermostat

##### outgoing

```
0x01, tt, hh, pp, b
tt = temperature
hh = humidity
pp = pressure
b = voltage / 10 - 100
```

---

##### actuator-valve

##### outgoing

```
'B', b
b = voltage / 10 - 100
```

##### incoming

```
0xde v tt
v = valve percent
tt = displayed temperature
```

---

#### radio gateway

##### frame types
| Frame type | Frame id |
|-----------------|------|
| FRAME_CONFIGURE | 0x90 |
| FRAME_CONFIGURED | 0x91 |
| FRAME_SENDPACKET | 0x92 |
| FRAME_PACKETSENT | 0x93 |
| FRAME_RECEIVEPACKET | 0x94 |
| FRAME_INIT | 0x95 |
| FRAME_HB | 0x96 |
| FRAME_ERR_INVALID_SIZE | 0x71 |
| FRAME_ERR_BUSY | 0x72 |
| FRAME_ERR_ADDR | 0x73 |
| FRAME_ERR_MEM | 0x74 |
| FRAME_ERR_TIMEOUT | 0x75 |

##### FRAME_CONFIGURE

(params may come in any order and combination)
```
K kkkkkkkkkkkkkkkk - encryption key (16 bytes)
F ff - frequency
N n - address (node ID / on old version it was network ID)
P p - power level
R rr - random seed
```