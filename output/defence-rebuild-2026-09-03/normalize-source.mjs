import fs from 'node:fs/promises';
import {FileBlob,PresentationFile} from '@oai/artifact-tool';
import {ROOT,OUT} from './content.mjs';
const p=await PresentationFile.importPptx(await FileBlob.load(`${ROOT}/docs/presentation/TFM_DEFENCE_FINAL_2026-09-03.pptx`));
const changes=[];
for(const [i,s]of p.slides.items.entries()){
 const l=JSON.parse(await(await s.export({format:'layout'})).text());
 for(const e of l.elements)if(e.kind==='shape'&&(e.bbox[2]<0||e.bbox[3]<0)){
  const sh=p.resolve(e.aid);const [x,y,w,h]=e.bbox;
  sh.position={left:x+Math.min(w,0),top:y+Math.min(h,0),width:Math.abs(w),height:Math.abs(h),horizontalFlip:w<0,verticalFlip:h<0};
  changes.push({slide:i+1,id:e.id,before:e.bbox,after:sh.position});
 }
}
await fs.writeFile(`${OUT}/import-normalization.json`,JSON.stringify(changes,null,2));
await(await PresentationFile.exportPptx(p)).save(`${OUT}/source-normalized.pptx`);
const s=p.slides.items[1],sh=s.shapes.items.find(x=>x.id==='10')||s.shapes.items[0];
console.log(JSON.stringify({changes,shapesMethods:Object.getOwnPropertyNames(Object.getPrototypeOf(s.shapes)),shapeMethods:Object.getOwnPropertyNames(Object.getPrototypeOf(sh)),style:sh.text.style,position:sh.position}));
