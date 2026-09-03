from pathlib import Path
from pypdf import PdfReader
import json,zipfile,xml.etree.ElementTree as ET
root=Path('C:/Users/Admin/PROJECTS/nao-ros4hri-bridge')
out=root/'output/defence-rebuild-2026-09-03'
r=PdfReader('C:/Users/Admin/Downloads/TFM_JUAN_BENDEK_FINAL (1).pdf')
(out/'thesis.txt').write_text('\n'.join(f'\n=== PDF PAGE {i+1} ===\n'+p.extract_text() for i,p in enumerate(r.pages)),encoding='utf-8')
(out/'assets').mkdir(exist_ok=True)
for n in [21,23,36,38,39,41,42]:
    p=r.pages[n-1]
    print(n,[(im.name, im.image.size) for im in p.images])
    for i,im in enumerate(p.images):
        (out/'assets'/f'thesis-{n}-{i}-{im.name}').write_bytes(im.data)
ns={'a':'http://schemas.openxmlformats.org/drawingml/2006/main','p':'http://schemas.openxmlformats.org/presentationml/2006/main'}
with zipfile.ZipFile(root/'docs/presentation/TFM_DEFENCE_FINAL_2026-09-03.pptx') as z:
    inv=[]
    for i in range(1,26):
        el=ET.fromstring(z.read(f'ppt/slides/slide{i}.xml'))
        shapes=[]
        for sp in el.findall('.//p:sp',ns):
            prop=sp.find('p:nvSpPr/p:cNvPr',ns)
            shapes.append({'shapeId':prop.get('id'),'name':prop.get('name'),'text':'\n'.join(t.text or '' for t in sp.findall('.//a:t',ns)),'placeholder':sp.find('.//p:ph',ns) is not None})
        inv.append({'slide':i,'shapes':shapes})
    (out/'source-xml-inventory.json').write_text(json.dumps(inv,ensure_ascii=False,indent=2),encoding='utf-8')
