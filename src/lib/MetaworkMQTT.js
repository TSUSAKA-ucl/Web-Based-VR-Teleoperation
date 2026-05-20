"use client";
import mqtt from 'mqtt'
import package_info from '../../package.json' // load version

console.log("Package_Info",package_info.name, package_info.version);
export const codeType = package_info.name; // software name
const version = package_info.version; // version number

// const MQTT_BROKER_URL = "wss://sora2.uclab.jp/mqws"; // For Nagoya-U UCLab Development
const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
// const protocol = "wss:";
const host = location.hostname;
// const port = 8333;  // WebSocketサーバーのポート
const port =  location.protocol === 'https:' ? 8333 : 9001;
const MQTT_BROKER_URL = `${protocol}//${host}:${port}`;

import {userUUID} from './cookie_id';

// global private variable
export var mqttclient = null;
export var idtopic = userUUID;

export const connectMQTT = (callback) => {
    if (mqttclient == null) {
      console.warn('MQTT_broker_url:',MQTT_BROKER_URL);
        const client = new mqtt.connect(MQTT_BROKER_URL, {protocolVersion: 5}); 
        client.on("connect", () => {
            console.log("MQTT Connected", client);

            const date = new Date();
            var devType = "browser";
            if(window.location.pathname.endsWith("/viewer/")) {
                devType = "robot";
            }
            const info = {
                date: date.toLocaleString(),
                device: {
                    //browser: navigator.appName,
                    //version: navigator.appVersion,
                    agent: navigator.userAgent,
                    //platform: navigator.platform,
                    cookie: navigator.cookieEnabled
                },
                devType: devType,
                codeType: codeType,
                version: version,
                devId: userUUID
            }
            // this is Metawork-MQTT protocol
            client.publish('mgr/register', JSON.stringify(info)) // for other devices.// maybe retransmit each 10seconds
            client.subscribe('dev/'+userUUID, {noLocal: true}, (err, granted) => {
                if (!err) {
                    console.log('MQTT Subscribe Granted',  granted);
                } else {
                    console.error('MQTT Subscription error: ', err);
                }
            });
            callback && callback(client);

        });
        client.on('error', function (err) {
            console.error('MQTT Connection error: ');
        });
        mqttclient = client;
    }
    return mqttclient
}


export const subscribeMQTT = (topic) => {
    if (mqttclient == null) {
        console.error('MQTT client not connected!');
        return;
    }
    mqttclient.subscribe(topic, {noLocal: true}, (err, granted) => {
        if (!err) {
            console.log('MQTT Subscribe topics', topic, granted);
        } else {
            console.error('MQTT Subscription error: ', err);
        }
    });
}

export const publishMQTT = (topic, msg) => {
    mqttclient.publish(topic, msg);
}
