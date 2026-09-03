import fs from 'node:fs/promises';
import { FileBlob, PresentationFile } from '@oai/artifact-tool';
const root = 'C:/Users/Admin/PROJECTS/nao-ros4hri-bridge';
const deck = await PresentationFile.importPptx(await FileBlob.load(`${root}/docs/presentation/TFM_DEFENCE_FINAL_2026-09-03.pptx`));
for (const [i, slide] of deck.slides.items.entries()) {
  const png = await deck.export({slide, format:'png', scale:1});
  await fs.writeFile(`${root}/output/presentation-review-build/pptx-${i+1}.png`,new Uint8Array(await png.arrayBuffer()));
  console.log(`Rendered ${i+1}`);
}
