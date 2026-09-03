import fs from 'node:fs/promises';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
export const ROOT='C:/Users/Admin/PROJECTS/nao-ros4hri-bridge';
export const OUT=`${ROOT}/output/defence-final-2026-09-03`;
export const SOURCE=`${ROOT}/docs/presentation/TFM_DEFENCE_REVISED_2026-09-03.pptx`;
export const FINAL=`${ROOT}/docs/presentation/TFM_DEFENCE_READY_2026-09-03.pptx`;
const raw=await fs.readFile('C:/Users/Admin/Downloads/TFM_DEFENCE_REVISED_ANNOTATED (1).html','utf8');
export const review=JSON.parse(raw.match(/<script id="embedded-review" type="application\/json">([\s\S]*?)<\/script>/)[1]);
export const deck=JSON.parse(raw.match(/<script id="deck-data" type="application\/json">([\s\S]*?)<\/script>/)[1]);
assert.equal(crypto.createHash('sha256').update(await fs.readFile(SOURCE)).digest('hex'),review.deckSha256);
assert.equal(review.slides.length,62);assert.equal(review.slides[24].decision,'Keep');assert.equal(review.slides[25].decision,'Keep');
export const edits={
  1:[{id:'18',crop:{left:.16,top:.30,right:.205,bottom:.18},reason:'User requested a closer in-frame crop of the original photograph; preserve title content and photo bytes.'}],
  13:[{id:'14',text:'object_id: canonical capability name.\nkind/category: skill and semantic family.\naliases: accepted compatibility names.\nparams: allowed argument names.\nrequired_params: mandatory subset.\n\nA plan invokes this capability with concrete args; the registry is its definition.',reason:'Remove the developing abstraction-level taxonomy, as requested globally.'}],
  17:[
    {id:'5',text:'IMPLEMENTATION · CONTRACTS 6–7',reason:'Focus this slide on result and feedback ownership.'},
    {id:'13',text:'Result → feedback → truthful closure',reason:'Avoid implying that the planner speaks.'},
    {id:'14',text:'C6 · skill result\nstatus + summary_text + evidence + failure + metadata.\n\nC7 · executor feedback\ngoal/plan/version + event + step + status/reason + result_payload.\n\nReporting boundary\nThe planner supplies structured status.\nDialogue Manager alone owns speaking.',reason:'Make the distinction between structured status and speaking explicit.'},
    {id:'15',text:'Action result → /planner/execution_feedback → planner supervision',reason:'Remove the requested planner-topic-to-dialogue-layer shorthand.'}
  ],
  21:[{id:'15',text:'I thoroughly reviewed, revised and tuned all agent-generated project code.',reason:'Express the approved supervision statement in the presenter’s first-person voice.'}],
  46:[{id:'16',text:'gold apple\nmultiturn',reason:'Native PowerPoint QA: wrap the accepted label to restore its gutter; preserve all words and slide structure.'}]
};
export const noteEdits={
  13:deck.slides[12].notes.replace('not every primitive or proposal is runtime-callable.','only configured runtime-callable skills are exposed.'),
  17:deck.slides[16].notes.replace('C7 carries result evidence, C8 requests communication and does not speak.','C7 carries result evidence back into planner supervision. Structured communication intents are internal API messages, never spoken output. The main slide deliberately avoids equating this message with speech.'),
  21:deck.slides[20].notes.replace('Authorship/supervision statement supplied directly by the user.','I supervised the agent-assisted development process and thoroughly reviewed, revised and tuned the agent-generated project code.')
};
await fs.mkdir(`${OUT}/template-inspect/layouts`,{recursive:true});
await fs.mkdir(`${OUT}/qa`,{recursive:true});
const inventory=[],outputSlides=[];
for(let n=1;n<=62;n++){
  const stem=`slide-${String(n).padStart(2,'0')}`;
  const txt=await fs.readFile(`${ROOT}/output/defence-rebuild-2026-09-03/final-layouts/${stem}.json`,'utf8');
  const layout=JSON.parse(txt);
  await fs.writeFile(`${OUT}/template-inspect/layouts/${stem}.json`,txt);
  for(const e of layout.elements)inventory.push({slide:n,...e,shapeId:e.id});
  outputSlides.push({outputSlide:n,sourceSlide:n,narrativeRole:edits[n]?deck.slides[n-1].title:'Preserve accepted source slide unchanged',sourceTitle:deck.slides[n-1].title,reuseMode:'duplicate-slide',editTargets:(edits[n]||[]).map(op=>{
    const e=layout.elements.find(e=>e.id===op.id);assert(e,`slide ${n}, ${op.id}`);
    return {shapeId:e.id,sourceElementId:e.id,sourceAnchor:e.aid,action:'rewrite',reason:op.reason};
  })});
}
await fs.writeFile(`${OUT}/template-inspect/template-inventory.ndjson`,inventory.map(x=>JSON.stringify(x)).join('\n'));
await fs.writeFile(`${OUT}/template-inspect/template-inspect.ndjson`,(await fs.readFile(`${SOURCE}.inspect.ndjson`,'utf8'))+'\n'+inventory.map(x=>JSON.stringify(x)).join('\n'));
await fs.writeFile(`${OUT}/template-frame-map.json`,JSON.stringify({outputSlides,omittedSourceSlides:[]},null,2));
await fs.writeFile(`${OUT}/edit-plan.json`,JSON.stringify({edits,noteEdits},null,2));
await fs.writeFile(`${OUT}/template-audit.txt`,'Identity clone/edit of the previously inspected 62-slide revised deck. Cached source renders and layouts are verified by the exact saved-review source SHA256. Keep the same empty master/layout hierarchy, all source elements, Georgia/Arial/Consolas typography and inherited spacing. No new slides or shapes. Four annotated slides plus one native-QA label wrap on46. Source slides25,26 and appendix wording/order are preserved. No unfilled structural placeholders.\n');
await fs.writeFile(`${OUT}/deviation-log.txt`,'Slide1 image crop, slide13 taxonomy paragraph removal, slide17 ownership shorthand, and slide21 first-person disclaimer change. Related notes corrected. Native QA also wraps the existing gold apple multiturn label on46 without changing words. No other audience-facing content, structure or data changes.\n');
await fs.writeFile(`${OUT}/source-notes.txt`,deck.slides.map(s=>`${s.number}. ${s.title}\n${s.notes}`).join('\n\n'));
await fs.writeFile(`${OUT}/annotation-audit.txt`,review.slides.map(s=>`${s.number}. ${s.decision||'Accepted by user for slides 33 onward'} | ${s.notes||'(no comment)'} | pins ${s.pins.length}`).join('\n'));
console.log('62-slide identity map; 4 annotated slides; latest 25 and 26 explicitly Keep.');
