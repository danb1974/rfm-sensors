import * as net from 'net';
import { concat, interval, Observable, of, Subject } from 'rxjs';
import { delay, distinctUntilChanged, map, retryWhen, startWith, switchMap, tap } from 'rxjs/operators';

import { Logger } from '../Logger';
import { timeSpan } from '../util';
import { ConnectableLayer } from './message';

// const FrameHeader = '\xDE\x5B';

export class Telnet implements ConnectableLayer<Buffer> {
    private readonly _data = new Subject<Buffer>();
    private socket: net.Socket | null = null;

    readonly data = this._data.asObservable();
    readonly connected = this.createSocket()
        .pipe(
            map(_ => true),
            startWith(false),
            switchMap(isConnected => {
                const fwd = of(isConnected);
                if (isConnected) {
                    return concat(fwd,
                        interval(timeSpan(3, 'min')).pipe(
                            // some data to keep telnet alive
                            switchMap(_ => this.send(Buffer.from([0xDE, 0x5B, 0x01, 0xFF, 0x40, 0x79]))),
                        )
                    );
                }
                return fwd;
            }),
            retryWhen(errs => errs.pipe(
                tap(err => this.logger.info(`telnet: error ${this.host}, trying to reconnect: ${err.message}`)),
                delay(this.reconnectInterval),
            )),
            distinctUntilChanged(),
        );

    constructor(
        private logger: Logger,
        private host: string,
        private port: number = 23,
        private reconnectInterval: number = timeSpan(5, 'sec'),
    ) {
    }

    private createSocket() {
        return new Observable<net.Socket>(observer => {
            const socket = new net.Socket();
            socket.setNoDelay(true);
            socket.on('data', data => this._data.next(data));
            socket.once('disconnect', () => observer.error(new Error('disconnected from server')));
            socket.once('error', err => observer.error(err));
            socket.once('end', (hadError: boolean) => {
                if (!hadError) {
                    observer.error(new Error('connection closed'));
                }
            });
            this.logger.info(`telnet: connecting to ${this.host}:${this.port}`);
            socket.connect(this.port, this.host, async () => {
                this.logger.info(`telnet: connected ${this.host}`);
                this.socket = socket;
                observer.next(socket);
            });
            return () => {
                this.socket = null;
                socket.end();
                socket.destroy();
            };
        });
    }

    send(data: Buffer): Observable<never> {
        return new Observable<never>(observer => {
            if (!this.socket) { throw new Error('telnet: socket not connected'); }
            this.socket.write(data, (err) => {
                if (err) { observer.error(err); } else { observer.complete(); }
            });
        });
    }
}
