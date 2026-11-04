// Test the browser packet adapter without browser/device dependencies.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
let pose = {transform: {position: {x: .1, y: 1, z: -.4}, orientation: {x:0,y:0,z:0,w:1}}};
const ref = {};
const input = {handedness:'right', gripSpace:{}};
const components = {};
const scene = {renderer:{xr:{getFrame:()=>({getPose:()=>pose}), getReferenceSpace:()=>ref,
                            getSession:()=>({inputSources:[input]})}}};
const sandbox = {document:{getElementById:id=>id==='scene'?scene:null, addEventListener:()=>{}},
                 AFRAME:{registerComponent:(name, value)=>{components[name]=value;}}, console};
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

// Progress must remain visible during a held grip and show server-authoritative timing.
const elements = new Map();
function element(id) {
  if (!elements.has(id)) elements.set(id, {attrs:{}, object3D:{visible:true},
    setAttribute(name,value){this.attrs[name]=value;}});
  return elements.get(id);
}
sandbox.document.getElementById = id => id==='scene' ? scene : element(id);
vm.runInContext(`status = {hardware:true, record:true, phase:'recording', title:'RECORDING',
  progress_text:'0:15 / 1:00 · 0:45 left', episode_progress:0.25,
  saved:3, target_episodes:50, episode:5, hint:'A: save', tracking:true}; renderStatus();`, sandbox);
assert.equal(element('timer').textContent, '0:15 / 1:00 · 0:45 left');
assert.equal(element('saved').textContent, '3 / 50');
assert.equal(element('episodeProgress').value, .25);
assert.equal(element('recordingHud').attrs.visible, true);
assert.match(element('recordingHudText').attrs.value, /RECORDING.*3\/50 saved/);
pose={transform:{position:{x:.1,y:1,z:-.4},orientation:{x:0,y:0,z:0,w:1}}};
hand.object3D.rotation={x:0,y:0,z:0};
hand.object3D.position={x:0,y:0,z:0};
components['teleop-stream'].tick.call({left:hand,right:hand,head:hand,updateAids:()=>{}});
assert.equal(element('rightHandHint').object3D.visible, true);
vm.runInContext('setConnection(false)',sandbox);
assert.match(element('recordingHudText').attrs.value, /DISCONNECTED/);
console.log('Persistent recording HUD, countdown, saved count and disconnect checks passed.');

// A burst of display updates must render only the latest, at bounded rate.
vm.runInContext(`statusDirty = true; status = {phase:'recording',record:true,saved:1};
  flushStatus(1000);`, sandbox);
assert.equal(element('saved').textContent, '1 / -');
vm.runInContext(`for (let i=2;i<=100;i++) {
  status = {phase:'recording',record:true,saved:i}; statusDirty = true; flushStatus(1050);
}`, sandbox);
assert.equal(element('saved').textContent, '1 / -');
vm.runInContext('flushStatus(1100)', sandbox);
assert.equal(element('saved').textContent, '100 / -');
vm.runInContext('statusDirty=true; setConnection(false); flushStatus(1200)', sandbox);
assert.match(element('recordingHudText').attrs.value, /DISCONNECTED/);

// Controller packets still go out on every XR frame; pose text is only 10 Hz.
const trackingFrames = [];
let frameClock = 2000;
sandbox.Date = {now:()=>frameClock};
sandbox.WebSocket = {OPEN:1};
sandbox.testSocket = {readyState:1,bufferedAmount:0,send:raw=>trackingFrames.push(JSON.parse(raw))};
vm.runInContext('ws=testSocket; statusDirty=false', sandbox);
let poseWrites = 0;
const poseInfo = element('headsetInfo');
const originalAttribute = poseInfo.setAttribute;
poseInfo.setAttribute = function(...args) {poseWrites++;originalAttribute.apply(this,args);};
const stream = {left:hand,right:hand,head:hand,updateAids:()=>{}};
for (let i=0;i<90;i++) {
  frameClock = 2000 + i*1000/90;
  components['teleop-stream'].tick.call(stream);
}
assert.equal(trackingFrames.length,90);
assert.ok(poseWrites<=10, `pose text wrote ${poseWrites} times`);
assert.equal(trackingFrames[89].sequence-trackingFrames[0].sequence,89);
console.log('Status bursts coalesce; controller streaming remains 90 Hz with at most 10 pose-text updates.');
