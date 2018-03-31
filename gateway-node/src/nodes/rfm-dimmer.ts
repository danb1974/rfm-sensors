import { RadioNode } from '../communication/node';

import * as moment from 'moment';
import { Observable } from 'rxjs/Observable';
import { Subject } from 'rxjs/Subject';

module.exports = function (RED) {

    function DimmerNode(config) {
        RED.nodes.createNode(this, config);
        const node = this;

        const bridge = RED.nodes.getNode(config.bridge);
        if (!bridge) { return; }

        const connected: Observable<boolean> = bridge.connected;
        const nodeLayer: RadioNode = bridge.create(parseInt(config.address, 10));
        const periodicSync = Observable.interval(5 * 60000).startWith(0);
        const stop = new Subject();

        let ledBrightness = config.ledbrightness;
        let mode = 0;
        if (config.manualdim) { mode |= 0x01; }
        if (!config.manual) { mode |= 0x02; }

        const syncState = () => Observable.concat(
            nodeLayer.send(Buffer.from([2])), // get status
            nodeLayer.send(Buffer.from([3, mode, config.maxbrightness])), // set mode
            nodeLayer.send(Buffer.from([4, ledBrightness])), // set led bright
        );

        Observable
            .combineLatest(connected, periodicSync)
            .filter(([isConnected]) => isConnected)
            .takeUntil(stop)
            .delayWhen(i => Observable.of(Math.round(Math.random() * 50) * 200))
            .subscribe(async ([isConnected, index]) => {
                try {
                    await syncState().toPromise();
                } catch (err) {
                    node.error(`while sync: ${err.message}`);
                }
            });

        const updateStatus = new Subject();
        let uploading = false;

        Observable
            .combineLatest(connected,
                nodeLayer.data.startWith(null).merge(updateStatus).timestamp(),
                Observable.interval(30000).startWith(0))
            .takeUntil(stop)
            .filter(() => !uploading)
            .subscribe(([isConnected, msg]) => {
                const lastMessage = msg.value ? `(${moment(msg.timestamp).fromNow()})` : '';

                node.status(isConnected
                    ? { fill: 'green', shape: 'dot', text: `connected ${lastMessage}` }
                    : { fill: 'red', shape: 'ring', text: 'not connected' });
            });

        const syncRequest = nodeLayer.data
            .filter(d => d.length === 1 && d[0] === 1)
            .takeUntil(stop)
            .subscribe(async () => {
                try {
                    await syncState().toPromise();
                } catch (err) {
                    node.error(`while sync: ${err.message}`);
                }
            });

        const dimmerBrightness = nodeLayer.data
            .filter(d => d.length === 2 && d[0] === 2)
            .map(d => d[1]);

        dimmerBrightness
            .takeUntil(stop)
            .subscribe(brightness => {
                node.send({
                    payload: brightness,
                    topic: config.topic,
                });
            });

        node.on('input', msg => {
            if (msg.type === 'firmware') {
                const hex = Buffer.isBuffer(msg.payload) ? msg.payload : Buffer.from(msg.payload);
                const progress = new Subject<number>();
                progress.throttleTime(1000).subscribe(p => {
                    node.status({ fill: 'green', shape: 'dot', text: `upload ${Math.round(p)} %` });
                });
                uploading = true;
                nodeLayer.upload(hex, progress)
                    .toPromise()
                    .then(() => {
                        node.status({ fill: 'green', shape: 'dot', text: `upload done!` });
                        return Observable.timer(1000).toPromise();
                    })
                    .catch(err => node.error(`while uploading hex: ${err.message}`))
                    .then(() => {
                        uploading = false;
                        updateStatus.next();
                    });
            } else {
                const value = Math.min(100, Math.max(0, parseInt(msg.payload, 10)));
                if (!isFinite(value)) {
                    return;
                }
                if (msg.type === 'led') {
                    ledBrightness = value;
                    // set led bright
                    nodeLayer
                        .send(Buffer.from([4, ledBrightness]))
                        .toPromise()
                        .catch(err => node.error(`while setting led bright: ${err.message}`));
                } else {
                    // set bright
                    nodeLayer
                        .send(Buffer.from([1, value]))
                        .toPromise()
                        .then(() => {
                            // fwd to output when succesful
                            node.send({
                                payload: value,
                                topic: config.topic,
                            });
                        })
                        .catch(err => node.error(`while setting bright: ${err.message}`));
                }
            }
        });

        node.on('close', () => {
            stop.next();
            stop.complete();
        });
    }

    RED.nodes.registerType('rfm-dimmer', DimmerNode);
};