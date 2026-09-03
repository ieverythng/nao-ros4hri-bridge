import fs from 'node:fs/promises';
import assert from 'node:assert/strict';
import {FileBlob,PresentationFile} from '@oai/artifact-tool';
import {OUT,FINAL,edits,noteEdits} from './plan.mjs';
process.on('uncaughtException',e=>{console.error(e.message);process.exit(1)});
process.on('unhandledRejection',e=>{console.error(e?.message||e);process.exit(1)});
const p=await PresentationFile.importPptx(await FileBlob.load(`${OUT}/template-starter.pptx`));
assert.equal(p.slides.items.length,62);
const changes=[];
for(const [index,slide]of p.slides.items.entries()){
  const number=index+1,stem=`slide-${String(number).padStart(2,'0')}`;
  const source=JSON.parse(await fs.readFile(`${OUT}/template-inspect/layouts/${stem}.json`,'utf8'));
  const layout=JSON.parse(await(await slide.export({format:'layout'})).text());
  for(const op of edits[number]||[]){
    const e=source.elements.find(x=>x.id===op.id);
    const ordinal=source.elements.filter(x=>x.kind===e.kind).findIndex(x=>x.id===e.id);
    const dest=layout.elements.filter(x=>x.kind===e.kind)[ordinal];
    const obj=p.resolve(dest.aid);
    if(op.text!==undefined){
      assert.equal(dest.text,e.text);
      obj.text=op.text;
      obj.text.style={...e.resolvedTextStyle,...e.paragraphs?.[0]?.resolvedTextStyle,autoFit:'none'};
    }
    if(op.crop){obj.crop=op.crop;console.log('Title crop',obj.crop);}
    changes.push({slide:number,id:dest.id,reason:op.reason});
  }
  if(noteEdits[number])slide.speakerNotes.textFrame.setText(noteEdits[number]);
}
await fs.mkdir(`${OUT}/rendered`,{recursive:true});await fs.mkdir(`${OUT}/layouts`,{recursive:true});
for(const [i,s]of p.slides.items.entries()){
  const stem=`slide-${String(i+1).padStart(2,'0')}`;
  await fs.writeFile(`${OUT}/layouts/${stem}.json`,await(await s.export({format:'layout'})).text());
  await fs.writeFile(`${OUT}/rendered/${stem}.png`,new Uint8Array(await(await p.export({slide:s,format:'png',scale:1})).arrayBuffer()));
}
await fs.writeFile(`${OUT}/edit-ledger.json`,JSON.stringify(changes,null,2));
await(await PresentationFile.exportPptx(p)).save(FINAL);
console.log(JSON.stringify({final:FINAL,slides:62,changes}));
