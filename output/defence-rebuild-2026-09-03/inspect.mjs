import fs from 'node:fs/promises';
import {FileBlob,PresentationFile} from '@oai/artifact-tool';
const base='C:/Users/Admin/PROJECTS/nao-ros4hri-bridge';
const out=`${base}/output/defence-rebuild-2026-09-03`;
const p=await PresentationFile.importPptx(await FileBlob.load(`${base}/docs/presentation/TFM_DEFENCE_FINAL_2026-09-03.pptx`));
await fs.mkdir(`${out}/template-inspect/layouts`,{recursive:true});
await fs.mkdir(`${out}/template-inspect/source-slides`,{recursive:true});
const snap=await p.inspect({kind:'deck,slide,textbox,shape,image,table,chart,notes,layout',maxChars:2000000});
await fs.writeFile(`${out}/template-inspect/template-inspect.ndjson`,snap.ndjson);
await fs.writeFile(`${out}/source-proto.json`,JSON.stringify(p.toProto(),null,2));
for(const [i,s]of p.slides.items.entries()){
  const stem=`slide-${String(i+1).padStart(2,'0')}`;
  await fs.writeFile(`${out}/template-inspect/layouts/${stem}.json`,await(await s.export({format:'layout'})).text());
  await fs.writeFile(`${out}/template-inspect/source-slides/${stem}.png`,new Uint8Array(await(await p.export({slide:s,format:'png',scale:1})).arrayBuffer()));
}
console.log(JSON.stringify({slides:p.slides.items.length,masters:p.masters.items.map(m=>({id:m.id,placeholders:m.placeholders.summary()})),layouts:p.layouts.items.map(l=>({id:l.id,placeholders:l.placeholders.summary()}))}));
