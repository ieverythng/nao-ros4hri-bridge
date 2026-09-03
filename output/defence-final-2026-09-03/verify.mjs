import fs from 'node:fs/promises';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import {FileBlob,PresentationFile} from '@oai/artifact-tool';
import {ROOT,OUT,FINAL,SOURCE,deck,edits,noteEdits} from './plan.mjs';
process.on('uncaughtException',e=>{console.error(e.message);process.exit(1)});
process.on('unhandledRejection',e=>{console.error(e?.message||e);process.exit(1)});
const hash=b=>crypto.createHash('sha256').update(b).digest('hex');
const p=await PresentationFile.importPptx(await FileBlob.load(FINAL));
assert.equal(p.slides.items.length,62);
await fs.mkdir(`${OUT}/final-rendered`,{recursive:true});await fs.mkdir(`${OUT}/final-layouts`,{recursive:true});
const changed=[],renderChanges=[],textEstimates=[],reviewSlides=[];
for(const [i,s]of p.slides.items.entries()){
  const n=i+1,stem=`slide-${String(n).padStart(2,'0')}`;
  const layoutText=await(await s.export({format:'layout'})).text(),layout=JSON.parse(layoutText);
  const source=JSON.parse(await fs.readFile(`${OUT}/template-inspect/layouts/${stem}.json`,'utf8'));
  const png=Buffer.from(await(await p.export({slide:s,format:'png',scale:1})).arrayBuffer());
  await fs.writeFile(`${OUT}/final-rendered/${stem}.png`,png);await fs.writeFile(`${OUT}/final-layouts/${stem}.json`,layoutText);
  const beforeText=source.elements.filter(e=>e.text).map(e=>e.text),afterText=layout.elements.filter(e=>e.text).map(e=>e.text);
  if(JSON.stringify(beforeText)!==JSON.stringify(afterText))changed.push(n);
  if(!edits[n])assert.deepEqual(afterText,beforeText,`Unexpected content change on accepted slide ${n}`);
  if(n>=33)assert.deepEqual(afterText.map(t=>t.replace(/\s+/g,' ')),beforeText.map(t=>t.replace(/\s+/g,' ')),`Appendix wording changed on ${n}`);
  if(hash(png)!==hash(await fs.readFile(`${ROOT}/output/defence-rebuild-2026-09-03/final-rendered/${stem}.png`)))renderChanges.push(n);
  assert(!/\bAB\s*[=≥]|abstraction.level/i.test(afterText.join('\n')),`Taxonomy on ${n}`);
  if(n===17)assert(!afterText.join('\n').includes('/planner/dialogue_act'));
  for(const e of layout.elements)if(e.text){
    if(e.text.trim().startsWith('{'))JSON.parse(e.text);
    const font=e.paragraphs?.[0]?.resolvedTextStyle?.fontSize||e.resolvedFontSize;
    const estimated=(e.textLayout?.lineCount||0)*font*1.15;
    if(estimated>e.bbox[3]+8)textEstimates.push({slide:n,id:e.id,height:e.bbox[3],estimated});
  }
  const native=await fs.readFile(`${OUT}/native-rendered/${stem}.png`);
  reviewSlides.push({...deck.slides[i],text:afterText.join('\n'),notes:noteEdits[n]||deck.slides[i].notes,image:`data:image/png;base64,${native.toString('base64')}`});
}
assert.deepEqual(changed,[13,17,21,46]);
const brief=['62 slides in the accepted order: 32 main, Q&A and reference appendix.','Latest annotations applied on slides 1, 13, 17 and 21; slides 25 and 26 explicitly preserved.','Appendix wording and order preserved; one label wraps for legibility on slide 46.','Native PowerPoint renders are embedded: this HTML works offline.'];
const data={source:FINAL.split('/').at(-1),sha256:hash(await fs.readFile(FINAL)),brief,slides:reviewSlides};
let html=await fs.readFile(`${ROOT}/docs/presentation/TFM_DEFENCE_REVISED_REVIEW_2026-09-03.html`,'utf8');
html=html.replace(/<script id="deck-data" type="application\/json">[\s\S]*?<\/script>/,`<script id="deck-data" type="application/json">${JSON.stringify(data).replaceAll('<','\\u003c')}</script>`)
 .replace(/<script id="embedded-review" type="application\/json">[\s\S]*?<\/script>/,'<script id="embedded-review" type="application/json">null</script>')
 .replaceAll('TFM_DEFENCE_REVISED_2026-09-03.pptx','TFM_DEFENCE_READY_2026-09-03.pptx')
 .replaceAll('TFM_DEFENCE_REVISED_ANNOTATED.html','TFM_DEFENCE_READY_ANNOTATED.html')
 .replaceAll('TFM_DEFENCE_REVISED_ANNOTATIONS','TFM_DEFENCE_READY_ANNOTATIONS')
 .replace('Revised deck · 32 main + 30 reference slides · review copy','Final deck · 32 main + 30 reference slides · offline copy')
 .replace('All 62 images are rendered from the exported revised PowerPoint.','All 62 images are rendered by Microsoft PowerPoint from the final exported deck.')
 .replace('The original PowerPoint and previous annotations are preserved.','The accepted structure, original PowerPoint and previous annotations are preserved.');
const script=html.match(/<script>\s*([\s\S]*?)<\/script>/)[1];new vm.Script(script);
const context=vm.createContext({document:{getElementById:()=>({textContent:JSON.stringify(data)})}});
vm.runInContext(script.slice(0,script.indexOf('let state=blank()')),context);
assert.equal(vm.runInContext('validate(blank()).slides.length',context),62);
for(const mutation of ['v.deckSha256="wrong"','v.slides.pop()','v.slides[0].decision="invalid"','v.slides[0].number=2','v.slides[0].pins=[{x:101,y:50,note:"bad"}]'])assert.throws(()=>vm.runInContext(`{const v=blank();${mutation};validate(v)}`,context));
const roundtrip=vm.runInContext('{const v=blank();v.slides[24].decision="Keep";v.slides[25].notes="Saved ✓ café <test>";v.slides[25].pins=[{x:20,y:40,note:"Check"}];JSON.stringify(validate(JSON.parse(JSON.stringify(v))))}',context);
assert.equal(JSON.parse(roundtrip).slides[25].notes,'Saved ✓ café <test>');
const htmlPath=FINAL.replace('.pptx','.html');await fs.writeFile(htmlPath,html);
const report={slides:62,changedTextSlides:changed,renderChanges,unchangedAcceptedSlides:true,allAppendixTextPreserved:true,allJsonParse:true,textEstimates,htmlStateValidation:true,sha256:data.sha256};
await fs.writeFile(`${OUT}/qa/final-check.json`,JSON.stringify(report,null,2));
console.log(JSON.stringify({...report,htmlPath}));
