import { RadioNode } from '../communication/node';
import { SerialLayer } from '../communication/serial';
import { PackageLayer } from './../communication/package';
import { RadioLayer } from './../communication/radio';
import { getBaseLayer } from './../util';

import { Observable } from 'rxjs/Observable';
import { ReplaySubject } from 'rxjs/ReplaySubject';

import '../vendor';

module.exports = function (RED) {
    function BridgeNode(config) {
        RED.nodes.createNode(this, config);

        const base = getBaseLayer(config.port, RED.log);
        const packageLayer = new PackageLayer(base);
        const radioLayer = new RadioLayer(packageLayer, RED.log);

        this.connected = base.connected
            .concatMap(isConnected => {
                if (isConnected) {
                    return radioLayer
                        .init({
                            key: Buffer.from(config.key, 'hex'),
                            powerLevel: 31
                        })
                        .map(() => isConnected)
                        .concat(Observable.of(isConnected))
                        .distinctUntilChanged();
                }
                return Observable.of(isConnected);
            })
            .catch(err => {
                this.error(`while initializing communication ${err.message}`);
                return Observable.empty();
            })
            .share();

        this.create = (address: number) => new RadioNode(radioLayer, address);

        this.on('close', () => {
            base.close();
            this.connected.complete();
        });

        base.connect();
    }

    RED.nodes.registerType('rfm-bridge', BridgeNode);
};
