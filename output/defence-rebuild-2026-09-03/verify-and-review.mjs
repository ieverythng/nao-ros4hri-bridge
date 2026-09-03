import fs from 'node:fs/promises';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import {FileBlob,PresentationFile} from '@oai/artifact-tool';
import {slides,ROOT,OUT} from './content.mjs';
process.on('uncaughtException',e=>{console.error(e.message);process.exit(1);});
process.on('unhandledRejection',e=>{console.error(e?.message||e);process.exit(1);});
const source=`${ROOT}/docs/presentation/TFM_DEFENCE_REVISED_2026-09-03.pptx`;
const hash=b=>crypto.createHash('sha256').update(b).digest('hex');
const p=await PresentationFile.importPptx(await FileBlob.load(source));
assert.equal(p.slides.items.length,slides.length);
await fs.mkdir(`${OUT}/final-rendered`,{recursive:true});await fs.mkdir(`${OUT}/final-layouts`,{recursive:true});
const review=[],changedRenders=[],textEstimates=[];
for(const [i,s]of p.slides.items.entries()){
 const stem=`slide-${String(i+1).padStart(2,'0')}`;
 const layoutText=await(await s.export({format:'layout'})).text(),l=JSON.parse(layoutText);
 const png=Buffer.from(await(await p.export({slide:s,format:'png',scale:1})).arrayBuffer());
 await fs.writeFile(`${OUT}/final-rendered/${stem}.png`,png);
 await fs.writeFile(`${OUT}/final-layouts/${stem}.json`,layoutText);
 if(hash(png)!==hash(await fs.readFile(`${OUT}/rendered/${stem}.png`)))changedRenders.push(i+1);
 for(const e of l.elements)if(e.text){
  const font=e.paragraphs?.[0]?.resolvedTextStyle?.fontSize||e.resolvedFontSize;
  const estimatedHeight=(e.textLayout?.lineCount||0)*font*1.15;
  if(estimatedHeight>e.bbox[3]+8)textEstimates.push({slide:i+1,id:e.id,text:e.text.slice(0,80),boxHeight:e.bbox[3],estimatedHeight});
 }
 const text=l.elements.filter(e=>e.text).map(e=>e.text).join('\n');
 review.push({number:i+1,title:i===0?'Design, Integration, and Validation of a Modular LLM-Based Interactive Architecture':slides[i].title,text,notes:slides[i].notes,width:1280,height:720,image:`data:image/png;base64,${png.toString('base64')}`});
}
assert.equal(hash(await fs.readFile(`${ROOT}/docs/presentation/TFM_DEFENCE_FINAL_2026-09-03.pptx`)),'6c99e8814d79907504d20267b44a4818c0b01e6ee462b1e2b14a799117e2ac22');
for(const s of slides)for(const op of s.ops)if(op.text?.trim().startsWith('{'))JSON.parse(op.text);
const brief=['32 main slides, including a two-minute demonstration; Q&A follows conclusion.','Thesis-aligned order: introduction, foundations, architecture, nodes/contracts, methodology, results, discussion, future work and conclusion.','Reference appendix retains questionnaires, payloads and evidence.','Keep simulated results, physical integration and primary/model-supplement denominators distinct.','The supervisor fallback video is not embedded because its file has not been supplied.'];
const data={source:source.split('/').at(-1),sha256:hash(await fs.readFile(source)),brief,slides:review};
let shell=await fs.readFile(`${ROOT}/output/presentation-review-build/review-shell.html`,'utf8');
shell=shell.replace('Original deck · 25 slides · review copy only','Revised deck · 32 main + 30 reference slides · review copy')
 .replace('All 25 slides','All 62 slides')
 .replace('Slide numbers always refer to the unchanged PowerPoint.','Slide numbers refer to the revised PowerPoint, not the original annotated deck.')
 .replace(/<ul><li>Bring back[\s\S]*?<\/ul>/,`<ul>${brief.map(x=>`<li>${x}</li>`).join('')}</ul>`)
 .replace(/Source: TFM_DEFENCE_FINAL_2026-09-03.pptx\.[\s\S]*?Animations are not reproduced\./,'Source: TFM_DEFENCE_REVISED_2026-09-03.pptx. All 62 images are rendered from the exported revised PowerPoint. Copyable text comes from that export; speaker notes include timing, evidence and source references. The original PowerPoint and previous annotations are preserved. Animations are not reproduced.')
 .replace('These are references for discussion, not decisions to insert or replace any slide.','These thesis diagrams inform the rebuilt technical core.')
 .replace("data.number<=19?'MAIN DECK':'APPENDIX'","data.number<=32?'MAIN DECK':data.number===33?'Q&A':'APPENDIX'")
 .replace('Original speaker notes','Speaker notes & sources')
 .replaceAll('TFM_DEFENCE_ANNOTATED.html','TFM_DEFENCE_REVISED_ANNOTATED.html')
 .replaceAll('TFM_DEFENCE_ANNOTATIONS','TFM_DEFENCE_REVISED_ANNOTATIONS');
const html=shell.replace('__DECK_DATA__',JSON.stringify(data).replaceAll('<','\\u003c'));
const script=html.match(/<script>\s*([\s\S]*?)<\/script>/)[1];new vm.Script(script);
const context=vm.createContext({document:{getElementById:()=>({textContent:JSON.stringify(data)})}});
vm.runInContext(script.slice(0,script.indexOf('let state=blank()')),context);
assert.equal(vm.runInContext('validate(blank()).slides.length',context),62);
for(const mutation of ['v.deckSha256="wrong"','v.slides.pop()','v.slides[0].decision="invalid"','v.slides[0].number=2','v.slides[0].pins=[{x:101,y:50,note:"bad"}]'])assert.throws(()=>vm.runInContext(`{const v=blank();${mutation};validate(v)}`,context));
assert.equal(vm.runInContext('{const v=blank();v.slides[0].decision="Keep";v.slides[0].notes="QA <script> café";v.slides[0].pins=[{x:25,y:40,note:"Diagram"}];validate(JSON.parse(JSON.stringify(v))).slides[0].pins[0].note}',context),'Diagram');
const htmlPath=`${ROOT}/docs/presentation/TFM_DEFENCE_REVISED_REVIEW_2026-09-03.html`;
await fs.writeFile(htmlPath,html);
await fs.writeFile(`${OUT}/qa/roundtrip-check.json`,JSON.stringify({slides:62,changedRenders,textEstimates,sourceHash:data.sha256,originalUnchanged:true,allJsonExamplesParse:true,reviewValidation:true},null,2));
console.log(JSON.stringify({slides:62,changedRenders,textEstimates,htmlPath,htmlBytes:(await fs.stat(htmlPath)).size}));
