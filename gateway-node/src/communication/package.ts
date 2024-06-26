import {defer, from} from 'rxjs';
import {filter, publishReplay, refCount, scan, share, switchMap} from 'rxjs/operators';

import { timeSpan } from '../util';
import { ConnectableLayer } from './message';

const FrameHeader1 = 0xDE, FrameHeader2 = 0x5B;

enum RxStatus {
    Idle, Header, Size, Data, Checksum1, Checksum2
}

interface State {
    status: RxStatus;
    buffer: Buffer | null;
    size: number;
    offset: number;
    chkSum: number;
    received: Buffer[];
    last: number;
}

export class PackageLayer implements ConnectableLayer<Buffer> {
    readonly data = this.below.data
        .pipe(
            scan((state: State, value: Buffer) => {
                this.processReceivedData(state, value);
                return state;
            }, {
                status: RxStatus.Idle,
                buffer: null,
                offset: 0,
                chkSum: 0,
                size: 0,
                last: 0,
                received: [],
            } as State),
            filter(v => v.received.length > 0),
            switchMap(v => {
                const buffers = v.received;
                v.received = [];
                // return of(buffers[0]);
                return from(buffers);
            }),
            share()
        );

    readonly connected = this.replayConnected
        ? this.below.connected.pipe(publishReplay(1), refCount())
        : this.below.connected;

    constructor(
        private below: ConnectableLayer<Buffer>,
        private replayConnected = false,
    ) {
    }

    private processReceivedData(state: State, rx: Buffer) {
        const now = new Date().getTime();
        if (now - state.last > timeSpan(.3, 'sec')) {
            state.status = RxStatus.Idle;
        }
        state.last = now;
        for (const data of rx) {
            switch (state.status) {
                case RxStatus.Idle:
                    if (data === FrameHeader1) { state.status = RxStatus.Header; }
                    break;
                case RxStatus.Header:
                    state.status = data === FrameHeader2 ? RxStatus.Size : RxStatus.Idle;
                    break;
                case RxStatus.Size:
                    state.size = data;
                    state.buffer = Buffer.alloc(data);
                    state.offset = 0;
                    state.status = RxStatus.Data;
                    break;
                case RxStatus.Data:
                    state.offset = state.buffer!.writeUInt8(data, state.offset);
                    if (state.offset === state.buffer!.length) { state.status = RxStatus.Checksum1; }
                    break;
                case RxStatus.Checksum1:
                    state.chkSum = data << 8;
                    state.status = RxStatus.Checksum2;
                    break;
                case RxStatus.Checksum2:
                    state.chkSum |= data;
                    const checksum = this.getChecksum(state.buffer!);
                    if (checksum === state.chkSum) {
                        state.received.push(state.buffer!);
                        state.buffer = null;
                    }
                    state.status = RxStatus.Idle;
                    break;

                default:
                    break;
            }
        }
    }

    private getChecksum(data: Buffer) {
        let checksum = 0x1021;

        const size = data.length;
        for (let i = 0; i <= size; i++) {
            const byte = i === 0 ? size : data[i - 1];
            const roll = (checksum & 0x8000) !== 0 ? true : false;
            checksum <<= 1;
            checksum &= 0xFFFF;
            if (roll) { checksum |= 1; }
            checksum ^= byte;
        }

        return checksum;
    }

    send(data: Buffer) {
        return defer(() => {
            const packet = Buffer.alloc(data.length + 5);
            let offset = packet.writeUInt8(FrameHeader1, 0);
            offset = packet.writeUInt8(FrameHeader2, offset);
            offset = packet.writeUInt8(data.length, offset);
            offset += data.copy(packet, offset, 0, data.length);
            const checksum = this.getChecksum(data);
            packet.writeUInt16BE(checksum, offset);
            return this.below.send(packet);
        });
    }
}
