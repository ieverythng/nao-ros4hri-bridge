import fs from 'node:fs';
import vm from 'node:vm';
import assert from 'node:assert/strict';
import crypto from 'node:crypto';
const root='C:/Users/Admin/PROJECTS/nao-ros4hri-bridge';
const html=fs.readFileSync(`${root}/docs/presentation/TFM_DEFENCE_REVIEW_2026-09-02.html`,'utf8');
const data=JSON.parse(html.match(/<script id="deck-data" type="application\/json">([\s\S]*?)<\/script>/)[1]);
const code=html.match(/<script>\s*([\s\S]*?)<\/script>/)[1];
new vm.Script(code);
assert.equal(data.slides.length,25);
assert.equal(data.sha256,crypto.createHash('sha256').update(fs.readFileSync(`${root}/docs/presentation/${data.source}`)).digest('hex'));
for(const s of data.slides){assert.equal(s.number,data.slides.indexOf(s)+1);assert(s.title?.trim());assert(s.text?.trim());const image=Buffer.from(s.image.split(',')[1],'base64');assert(image.equals(fs.readFileSync(`${root}/output/presentation-review-build/pptx-${s.number}.png`)));console.log(s.number,s.title);}
const context=vm.createContext({document:{getElementById:()=>({textContent:JSON.stringify(data)})}});
vm.runInContext(code.slice(0,code.indexOf('let state=blank()')),context);
assert.equal(vm.runInContext('validate(blank()).slides.length',context),25);
for(const mutation of ['v.deckSha256="wrong"','v.slides.pop()','v.slides[0].decision="invalid"','v.slides[0].number=2','v.slides[0].pins=[{x:101,y:50,note:"bad"}]']){
  assert.throws(()=>vm.runInContext(`{const v=blank();${mutation};validate(v)}`,context));
}
assert.equal(vm.runInContext('{const v=blank();v.slides[0].decision="Keep";v.slides[0].notes="Test <script> & Unicode: café";v.slides[0].pins=[{x:25,y:40,note:"Diagram"}];validate(JSON.parse(JSON.stringify(v))).slides[0].pins[0].note}',context),'Diagram');
assert(!html.includes('__DECK_DATA__'));
assert(!/<(?:script|link)[^>]+(?:src|href)="https?:/i.test(html));
console.log('PASS: JS syntax, 25 original image byte matches, slide identifiers, source hash, valid annotation roundtrip, invalid import rejection, self-contained assets.');
