import fs from 'node:fs/promises';
import {FileBlob,PresentationFile} from '@oai/artifact-tool';
import {slides,OUT,ROOT} from './content.mjs';
process.on('uncaughtException',e=>{console.error(e.message);process.exit(1);});
process.on('unhandledRejection',e=>{console.error(e?.message||e);process.exit(1);});
const p=await PresentationFile.importPptx(await FileBlob.load(`${OUT}/template-starter.pptx`));
const pos=b=>({left:b[0],top:b[1],width:b[2],height:b[3]});
const styles=e=>({...e.resolvedTextStyle,...e.paragraphs?.[0]?.resolvedTextStyle});
const ledger=[];
for(const [i,spec]of slides.entries()){
 const s=p.slides.items[i];
 const source=JSON.parse(await fs.readFile(`${OUT}/template-inspect/layouts/slide-${String(spec.src).padStart(2,'0')}.json`,'utf8'));
 const current=JSON.parse(await(await s.export({format:'layout'})).text());
 const mapped=new Map();
 for(const e of source.elements){
  const peers=source.elements.filter(x=>x.kind===e.kind),index=peers.findIndex(x=>x.id===e.id);
  const dest=current.elements.filter(x=>x.kind===e.kind)[index];
  if(!dest)throw Error(`Missing mapped ${i+1}:${e.kind}:${e.id}`);
  if(e.kind==='shape'&&(e.text||'')!==(dest.text||''))throw Error(`Mapping text mismatch ${i+1}:${e.id}`);
  mapped.set(e.id,{source:e,current:dest,obj:p.resolve(dest.aid)});
 }
 for(const op of spec.ops){
  if(op.action==='add'){
   const sh=s.shapes.add({geometry:'textbox',position:pos(op.pos),fill:'none',line:{fill:'none',width:0}});
   sh.text=op.text;sh.text.style={...op.style,autoFit:'none'};continue;
  }
  const item=mapped.get(op.id);if(!item)throw Error(`Unmapped ${i+1}:${op.id}`);
  const obj=item.obj;
  if(op.action==='delete'||op.action==='replace'){
   if(item.source.kind==='chart')s.charts.deleteById(obj.id);else obj.delete();
   if(op.image){
    const isPhoto=/Photo/.test(op.image),isTitle=i===0;
    const opts={blob:new Uint8Array(await fs.readFile(op.image)),contentType:isPhoto?'image/jpeg':'image/png',position:pos(op.pos),fit:isPhoto?'cover':'contain',alt:spec.title};
    // A PowerPoint crop, not an altered photograph: focus on the standing robot.
    if(isTitle)opts.crop={left:0.22,top:0.39,right:0.37,bottom:0.20};
    const im=s.images.add(opts);if(isTitle)im.crop=opts.crop;
   }
  }else{
   if(op.pos)obj.position=pos(op.pos);
   if(op.text!==undefined){obj.text=op.text;obj.text.style={...styles(item.source),...op.style,autoFit:'none'};}
   if(op.pos&&((spec.src===2&&['7','11'].includes(op.id))||(spec.src===17&&['7','11','15'].includes(op.id)))){
    obj.fill='none';obj.line={fill:'none',width:0};
   }
  }
  ledger.push({slide:i+1,sourceSlide:spec.src,sourceId:op.id,outputId:obj.id,action:op.action});
 }
 if(spec.src===10){
  const inheritedChart=s.charts.items[0];s.charts.deleteById(inheritedChart.id);
  const cats=['End-to-end 20/22','Scene 11/11','Stateful KB 7/7','Multi-step 7/9','Robustness 12/15','Capability 5/7','Nav recovery 1/1'];
  s.charts.add('bar',{position:{left:72,top:211,width:665,height:380},categories:cats,
   series:[{name:'Pass rate',values:[90.9,100,100,77.8,80,71.4,100],fill:'#0C6570'}],
   barOptions:{direction:'bar',grouping:'clustered',gapWidth:45},hasLegend:false,
   xAxis:{visible:true,textStyle:{fontSize:17,fill:'#5B6770'},majorGridlines:null},
   yAxis:{min:0,max:100,majorUnit:20,numberFormatCode:'0"%"',textStyle:{fontSize:15,fill:'#073F47'},majorGridlines:{fill:'#DDE4E4',width:0.5}},
   dataLabels:{showValue:false},chartFill:'none',plotAreaFill:'none'});
 }
 if(spec.src===20){
  const t=s.tables.items[0];const vals=[['Suite','Pass','Non-pass','Total / pass rate'],['End-to-end',20,2,'22 cases · 90.9%'],['Scene / grounding',11,0,'11 cases · 100%'],['Stateful KB',7,0,'7 cases · 100%'],['Multi-step',7,2,'9 cases · 77.8%'],['Robustness',12,3,'15 cases · 80.0%'],['Multi-capability',5,2,'7 cases · 71.4%'],['Navigation recovery',1,0,'1 case · 100%']];
  for(let r=0;r<vals.length;r++)for(let c=0;c<vals[r].length;c++){
   const cell=t.getCell(r,c);cell.value=String(vals[r][c]);cell.text.style={fontSize:19,typeface:'Arial',color:r===0?'#FFFFFF':'#073F47',bold:r===0};
  }
 }
 s.speakerNotes.textFrame.setText(spec.notes);
}
await fs.mkdir(`${OUT}/rendered`,{recursive:true});await fs.mkdir(`${OUT}/layouts`,{recursive:true});
await fs.writeFile(`${OUT}/edit-ledger.json`,JSON.stringify(ledger,null,2));
await fs.writeFile(`${OUT}/final-proto.json`,JSON.stringify(p.toProto(),null,2));
for(const [i,s]of p.slides.items.entries()){
 const stem=`slide-${String(i+1).padStart(2,'0')}`;
 await fs.writeFile(`${OUT}/layouts/${stem}.json`,await(await s.export({format:'layout'})).text());
 await fs.writeFile(`${OUT}/rendered/${stem}.png`,new Uint8Array(await(await p.export({slide:s,format:'png',scale:1})).arrayBuffer()));
}
const final=`${ROOT}/docs/presentation/TFM_DEFENCE_REVISED_2026-09-03.pptx`;
await(await PresentationFile.exportPptx(p)).save(final);
console.log(JSON.stringify({final,slides:slides.length,bytes:(await fs.stat(final)).size}));
