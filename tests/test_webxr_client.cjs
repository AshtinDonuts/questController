// Test the browser packet adapter without browser/device dependencies.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
let pose = {transform: {position: {x: .1, y: 1, z: -.4}, orientation: {x:0,y:0,z:0,w:1}}};
const ref = {};
const input = {handedness:'right', gripSpace:{}};
const scene = {renderer:{xr:{getFrame:()=>({getPose:()=>pose}), getReferenceSpace:()=>ref,
                            getSession:()=>({inputSources:[input]})}}};
const sandbox = {document:{getElementById:id=>id==='scene'?scene:null, addEventListener:()=>{}},
                 AFRAME:{registerComponent:()=>{}}, console};
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync('webxr_teleop/web/app.js','utf8'), sandbox);
const gamepad = {buttons:Array.from({length:6},()=>({pressed:false,value:0}))};
gamepad.buttons[1].pressed=true;
const hand = {object3D:{position:{x:99,y:99,z:99}}, components:{'tracked-controls':{controller:{gamepad}}}};
sandbox.hand=hand;
let result=vm.runInContext("readController(hand, 'right')",sandbox);
assert.equal(result.position.x,.1); // Must use XRFrame pose, not cached object/model transform.
assert.equal(result.tracked,true);
assert.equal(result.buttons.squeeze,true);
pose=null;
assert.equal(vm.runInContext("readController(hand, 'right')",sandbox),null);
pose={emulatedPosition:true,transform:{position:{},orientation:{}}};
assert.equal(vm.runInContext("readController(hand, 'right')",sandbox),null);
console.log('Client tracked-pose, grip, tracking-loss and emulated-pose checks passed.');
// A reset persists in the next packet even when the immediate send was suppressed.
vm.runInContext('resetReference(); resetReference();', sandbox);
assert.equal(vm.runInContext('referenceEpoch',sandbox),2);
console.log('Reference-space epoch persists while disconnected.');
