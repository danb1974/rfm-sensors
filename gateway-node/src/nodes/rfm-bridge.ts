import { RadioNode } from '../communication/node';
import { SerialLayer } from '../communication/serial';
import { PackageLayer } from './../communication/package';
import { RadioLayer } from './../communication/radio';

import { ReplaySubject } from 'rxjs/ReplaySubject';

import '../vendor';

module.exports = function (RED) {
    function BridgeNode(config) {
        RED.nodes.createNode(this, config);
        const node = this;

        const serial = new SerialLayer(config.port);
        const packageLayer = new PackageLayer(serial);
        const radioLayer = new RadioLayer(packageLayer);

        node.connected = new ReplaySubject<boolean>(1);

        serial
            .open()
            .then(() => radioLayer
                .init({
                    key: Buffer.from(config.key, 'hex'),
                    powerLevel: 31
                })
                .toPromise())
            .then(() => node.connected.next(true))
            .catch(err => node.error(`while initializing communication ${err.message}`));

        node.create = (address: number) => new RadioNode(radioLayer, address);

        node.on('close', () => {
            serial.close().catch(err => { });
            node.connected.complete();
        });
    }

    RED.nodes.registerType('rfm-bridge', BridgeNode);
};