"""Package original PPTX renders and extracted text as a portable review document."""
import base64
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
BUILD = Path(__file__).resolve().parent
SOURCE = ROOT / 'docs/presentation/TFM_DEFENCE_FINAL_2026-09-03.pptx'
OUTPUT = ROOT / 'docs/presentation/TFM_DEFENCE_REVIEW_2026-09-02.html'
NS = {'a': 'http://schemas.openxmlformats.org/drawingml/2006/main',
      'p': 'http://schemas.openxmlformats.org/presentationml/2006/main'}

def paragraphs(xml):
    return [''.join(n.itertext()) for n in ET.fromstring(xml).findall('.//a:p', NS)]

with zipfile.ZipFile(SOURCE) as archive:
    slides = []
    # The source deck's relationship order is verified rather than relying on filenames.
    presentation = ET.fromstring(archive.read('ppt/presentation.xml'))
    relationships = ET.fromstring(archive.read('ppt/_rels/presentation.xml.rels'))
    targets = {r.attrib['Id']: r.attrib['Target'] for r in relationships}
    for number, item in enumerate(presentation.findall('p:sldIdLst/p:sldId', NS), 1):
        rel = item.attrib['{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id']
        target = targets[rel]
        name = target.lstrip('/') if target.startswith('/') else 'ppt/' + target
        text = paragraphs(archive.read(name))
        notes_name = f'ppt/notesSlides/notesSlide{number}.xml'
        notes = '\n'.join(paragraphs(archive.read(notes_name))) if notes_name in archive.namelist() else ''
        image = BUILD / f'pptx-{number}.png'
        with Image.open(image) as im:
            width, height = im.size
        title = text[2] if number <= 19 else text[3]
        slides.append({'number': number, 'sourcePart': name, 'title': title,
                       'text': '\n'.join(text), 'notes': notes,
                       'width': width, 'height': height,
                       'image': 'data:image/png;base64,' + base64.b64encode(image.read_bytes()).decode()})

assert len(slides) == 25
brief = [
    'Reuse the existing main architecture, message-flow, and runtime-flow diagrams.',
    'Prefer a direct scientific-defence tone over pitch-like language.',
    'Follow the thesis: literature/context, architecture/implementation, evaluation/traces, results, discussion/limitations, conclusion.',
    'Preserve useful cues for oral explanation; use per-slide Keep/Change/Remove/Refactor decisions.',
    'Discuss annotations using grill-me one decision at a time before revising the PPT; apply iiia-ros4hri-check to ownership and evidence claims.',
    'Use the supplied TFM_JUAN_BENDEK_FINAL (1).pdf (July 23, 2026; 76 pages), read in full, rather than the older local thesis draft.'
]
data = {'source': SOURCE.name, 'sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        'brief': brief, 'slides': slides}
payload = json.dumps(data, ensure_ascii=False).replace('<', '\\u003c')
shell = (BUILD / 'review-shell.html').read_text(encoding='utf-8')
assert shell.count('__DECK_DATA__') == 1
OUTPUT.write_text(shell.replace('__DECK_DATA__', payload), encoding='utf-8')
print(f'Created {OUTPUT}: {len(slides)} slides, {OUTPUT.stat().st_size:,} bytes')
