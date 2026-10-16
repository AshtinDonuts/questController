const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../webxr_teleop/web/app.js'), 'utf8');
function fixture(supported=true, fail=false) {
  const button = {disabled:true}, note = {}, probes=[];
  let vr=0, ar=0;
  const scene = {enterVR(){vr++;if(fail)throw Error('session rejected');return Promise.resolve();}, enterAR(){ar++;}};
  const context=vm.createContext({navigator:{xr:{isSessionSupported:async mode=>{probes.push(mode);return supported;}}},
    document:{getElementById:id=>({enterXr:button,xrNote:note,scene})[id],addEventListener(){}},
    AFRAME:{registerComponent(){}}, window:{}, console});
  vm.runInContext(source,context);
  return {context,button,note,probes,counts:()=>({vr,ar})};
}
test('requests VR even when AR might be supported',async()=>{
  const f=fixture();await f.context.setupEnterButton();
  assert.deepEqual(f.probes,['immersive-vr']);assert.equal(f.button.textContent,'Enter VR');
  await f.button.onclick();assert.deepEqual(f.counts(),{vr:1,ar:0});
});
test('failed entry enables retry and reports VR failure',async()=>{
  const f=fixture(true,true);await f.context.setupEnterButton();await f.button.onclick();
  assert.equal(f.button.disabled,false);assert.match(f.note.textContent,/Could not enter VR: session rejected/);
});
test('unsupported VR stays disabled',async()=>{
  const f=fixture(false);await f.context.setupEnterButton();assert.equal(f.button.disabled,true);
});
