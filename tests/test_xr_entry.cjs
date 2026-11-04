const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../webxr_teleop/web/app.js'), 'utf8');
function fixture(arSupported=true, vrSupported=true, fail=false) {
  const button = {disabled:true}, vrButton={disabled:true}, note = {}, probes=[];
  let vr=0, ar=0;
  const scene = {
    enterVR(){vr++;if(fail)throw Error('session rejected');return Promise.resolve();},
    enterAR(){ar++;if(fail)throw Error('session rejected');return Promise.resolve();},
  };
  const context=vm.createContext({navigator:{xr:{isSessionSupported:async mode=>{
    probes.push(mode);return mode==='immersive-ar' ? arSupported : vrSupported;
  }}}, document:{getElementById:id=>({enterXr:button,enterVr:vrButton,xrNote:note,scene})[id],addEventListener(){}},
    AFRAME:{registerComponent(){}}, window:{}, console});
  vm.runInContext(source,context);
  return {context,button,vrButton,note,probes,counts:()=>({vr,ar})};
}
test('primary entry requests passthrough and preserves explicit VR choice',async()=>{
  const f=fixture();await f.context.setupEnterButton();
  assert.deepEqual(f.probes,['immersive-ar','immersive-vr']);
  assert.equal(f.button.textContent,'Enter passthrough');
  await f.button.onclick();assert.deepEqual(f.counts(),{vr:0,ar:1});
  await f.vrButton.onclick();assert.deepEqual(f.counts(),{vr:1,ar:1});
});
test('failed passthrough enables retry without silently entering VR',async()=>{
  const f=fixture(true,true,true);await f.context.setupEnterButton();await f.button.onclick();
  assert.equal(f.button.disabled,false);
  assert.match(f.note.textContent,/Could not enter passthrough: session rejected/);
  assert.deepEqual(f.counts(),{vr:0,ar:1});
});
test('unsupported passthrough leaves explicit VR available',async()=>{
  const f=fixture(false,true);await f.context.setupEnterButton();
  assert.equal(f.button.disabled,true);assert.equal(f.vrButton.disabled,false);
  assert.match(f.note.textContent,/VR hides the real room/);
});
test('passthrough works even when VR is unavailable',async()=>{
  const f=fixture(true,false);await f.context.setupEnterButton();
  assert.equal(f.button.disabled,false);assert.equal(f.vrButton.disabled,true);
  await f.button.onclick();assert.deepEqual(f.counts(),{vr:0,ar:1});
});
test('no supported immersive mode leaves both entry buttons disabled',async()=>{
  const f=fixture(false,false);await f.context.setupEnterButton();
  assert.equal(f.button.disabled,true);assert.equal(f.vrButton.disabled,true);
});
test('no WebXR reports how to open the page',async()=>{
  const f=fixture();f.context.navigator.xr=undefined;await f.context.setupEnterButton();
  assert.equal(f.button.disabled,true);assert.equal(f.vrButton.disabled,true);
  assert.match(f.note.textContent,/Meta Quest browser/);
});
