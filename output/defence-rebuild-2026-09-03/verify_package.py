"""Read-only package verification; never modifies a PowerPoint archive."""
from pathlib import Path
from zipfile import ZipFile
import hashlib,json,re,posixpath
import xml.etree.ElementTree as ET

ROOT=Path('C:/Users/Admin/PROJECTS/nao-ros4hri-bridge')
OUT=ROOT/'output/defence-rebuild-2026-09-03'
PPTX=ROOT/'docs/presentation/TFM_DEFENCE_REVISED_2026-09-03.pptx'
NS={'a':'http://schemas.openxmlformats.org/drawingml/2006/main','p':'http://schemas.openxmlformats.org/presentationml/2006/main'}
with ZipFile(PPTX,'r') as archive:
    names=set(archive.namelist())
    assert archive.testzip() is None
    slides=sorted(n for n in names if re.fullmatch(r'ppt/slides/slide\d+\.xml',n))
    notes=[n for n in names if re.fullmatch(r'ppt/notesSlides/notesSlide\d+\.xml',n)]
    assert len(slides)==62 and len(notes)==62
    for name in names:
        if name.endswith(('.xml','.rels')): ET.fromstring(archive.read(name))
        if name.endswith('.rels'):
            directory=posixpath.dirname(posixpath.dirname(name))
            for rel in ET.fromstring(archive.read(name)):
                if rel.get('TargetMode')=='External':continue
                target=rel.get('Target','')
                resolved=target.lstrip('/') if target.startswith('/') else posixpath.normpath(posixpath.join(directory,target))
                assert resolved in names,(name,resolved)
    for name in slides:
        root=ET.fromstring(archive.read(name))
        for extent in root.findall('.//a:ext',NS):
            if 'cx' in extent.attrib: assert int(extent.get('cx'))>=0 and int(extent.get('cy'))>=0
        text='\n'.join(t.text or '' for t in root.findall('.//a:t',NS))
        assert not re.search(r'click to add|lorem ipsum|text goes here',text,re.I)
    for name in notes:
        text='\n'.join(t.text or '' for t in ET.fromstring(archive.read(name)).findall('.//a:t',NS))
        assert '[Sources]' in text,name
    charts=[n for n in names if '/charts/' in n and '/_rels/' not in n and n.endswith('.xml')]
    assert len(charts)==1
    report={'status':'pass','slideCount':len(slides),'notesWithSources':len(notes),'validXml':True,'allInternalRelationshipsResolve':True,'nonnegativeShapeExtents':True,'noTemplatePlaceholders':True,'nativeCharts':len(charts),'sha256':hashlib.sha256(PPTX.read_bytes()).hexdigest()}
(OUT/'qa/package-check.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report))
