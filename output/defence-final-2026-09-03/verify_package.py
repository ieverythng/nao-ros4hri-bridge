"""Read-only package/PDF inspection. Never writes or patches an Office archive."""
from pathlib import Path
from zipfile import ZipFile
import hashlib,json,re,posixpath
import xml.etree.ElementTree as ET
from pypdf import PdfReader
ROOT=Path('C:/Users/Admin/PROJECTS/nao-ros4hri-bridge')
OUT=ROOT/'output/defence-final-2026-09-03'
PPTX=ROOT/'docs/presentation/TFM_DEFENCE_READY_2026-09-03.pptx'
NS={'a':'http://schemas.openxmlformats.org/drawingml/2006/main','p':'http://schemas.openxmlformats.org/presentationml/2006/main'}
with ZipFile(PPTX,'r') as z:
    names=set(z.namelist());assert z.testzip() is None
    slides=[n for n in names if re.fullmatch(r'ppt/slides/slide\d+\.xml',n)]
    notes=[n for n in names if re.fullmatch(r'ppt/notesSlides/notesSlide\d+\.xml',n)]
    assert len(slides)==len(notes)==62
    for name in names:
        if name.endswith(('.xml','.rels')):ET.fromstring(z.read(name))
        if name.endswith('.rels'):
            directory=posixpath.dirname(posixpath.dirname(name))
            for rel in ET.fromstring(z.read(name)):
                if rel.get('TargetMode')=='External':continue
                target=rel.get('Target','')
                resolved=target.lstrip('/') if target.startswith('/') else posixpath.normpath(posixpath.join(directory,target))
                assert resolved in names,(name,resolved)
    for name in slides:
        root=ET.fromstring(z.read(name))
        for extent in root.findall('.//a:ext',NS):
            if 'cx' in extent.attrib:assert int(extent.get('cx'))>=0 and int(extent.get('cy'))>=0
        for shape in root.findall('.//p:sp',NS):
            if shape.find('.//p:ph',NS) is not None:assert ''.join(t.text or '' for t in shape.findall('.//a:t',NS)).strip()
        text='\n'.join(t.text or '' for t in root.findall('.//a:t',NS))
        assert not re.search(r'click to add|lorem ipsum|text goes here|\bAB\s*[=≥]',text,re.I)
    for name in notes:assert '[Sources]' in ''.join(t.text or '' for t in ET.fromstring(z.read(name)).findall('.//a:t',NS))
    charts=[n for n in names if '/charts/' in n and '/_rels/' not in n and n.endswith('.xml')];assert len(charts)==1
pdf=PdfReader(PPTX.with_suffix('.pdf'));assert len(pdf.pages)==62
page_text=[p.extract_text() or '' for p in pdf.pages]
assert all(len(t.strip())>25 for t in page_text)
assert 'thoroughly reviewed' in page_text[20]
assert 'AB=1' not in '\n'.join(page_text)
assert '/planner/dialogue_act' not in page_text[16]
font_records={}
for p in pdf.pages:
    assert abs(float(p.mediabox.width)/float(p.mediabox.height)-16/9)<.001
    for ref in p['/Resources'].get('/Font',{}).values():
        font=ref.get_object()
        descendants=font.get('/DescendantFonts',[font])
        for child in descendants:
            actual=child.get_object();descriptor=actual.get('/FontDescriptor')
            if descriptor:
                descriptor=descriptor.get_object()
                embedded=any(k in descriptor for k in ['/FontFile','/FontFile2','/FontFile3'])
                font_records[str(actual.get('/BaseFont'))]=embedded
assert font_records and all(font_records.values()),font_records
report={'status':'pass','slideCount':62,'notesWithSources':62,'allRelationshipsResolve':True,'nonnegativeExtents':True,'noEmptyPlaceholders':True,'nativeCharts':len(charts),'pdfPages':62,'allPdfPagesSearchable':True,'pdfFontsEmbedded':font_records,'aspectRatio':'16:9','sha256':hashlib.sha256(PPTX.read_bytes()).hexdigest()}
(OUT/'qa/package-check.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report))
