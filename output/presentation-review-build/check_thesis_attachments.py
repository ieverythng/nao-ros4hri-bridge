import collections
import csv
import hashlib
import io
from pathlib import Path
from pypdf import PdfReader

source = Path('C:/Users/Admin/Downloads/TFM_JUAN_BENDEK_FINAL (1).pdf')
reader = PdfReader(source)
print('Thesis SHA256:', hashlib.sha256(source.read_bytes()).hexdigest())
for number, page in enumerate(reader.pages, 1):
    for reference in page.get('/Annots', []):
        annotation = reference.get_object()
        if annotation.get('/Subtype') != '/FileAttachment':
            continue
        spec = annotation['/FS']
        blob = spec['/EF']['/F'].get_data()
        rows = list(csv.DictReader(io.StringIO(blob.decode('utf-8-sig'))))
        print('Page', number, spec.get('/F'), 'bytes', len(blob), 'SHA256', hashlib.sha256(blob).hexdigest())
        print('Rows', len(rows), 'Columns', len(rows[0]), 'Statuses', collections.Counter(r.get('status', '') for r in rows))
        groups = collections.defaultdict(collections.Counter)
        for row in rows:
            groups[row.get('run_group', row.get('model', ''))][row.get('status', '')] += 1
        print('Groups', dict(groups))
