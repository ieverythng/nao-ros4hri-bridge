(() => {
  const main = document.querySelector('main.doc');
  if (!main || main.dataset.enhanced === 'true') return;
  main.dataset.enhanced = 'true';

  const children = [...main.children];
  const title = children.find((node) => node.tagName === 'H1');
  const firstSection = children.findIndex((node) => node.tagName === 'H2');
  if (!title || firstSection < 0) return;

  const hero = document.createElement('header');
  hero.className = 'harness-hero';
  const heroInner = document.createElement('div');
  heroInner.className = 'hero-inner';
  const eyebrow = document.createElement('div');
  eyebrow.className = 'eyebrow';
  eyebrow.textContent = 'Neural Workbench · Agentic Harness Research';
  const meta = document.createElement('div');
  meta.className = 'hero-meta';
  heroInner.append(eyebrow, title, meta);
  children.slice(1, firstSection).forEach((node) => meta.append(node));
  hero.append(heroInner);
  document.body.insertBefore(hero, main);

  const layout = document.createElement('div');
  layout.className = 'harness-layout';
  const toc = document.createElement('nav');
  toc.className = 'harness-toc';
  toc.setAttribute('aria-label', 'Document contents');
  const tocTitle = document.createElement('div');
  tocTitle.className = 'toc-title';
  tocTitle.textContent = 'Contents';
  toc.append(tocTitle);
  const content = document.createElement('article');
  content.className = 'harness-content';
  layout.append(toc, content);
  main.append(layout);

  let section = null;
  children.slice(firstSection).forEach((node) => {
    if (node.tagName === 'H2') {
      section = document.createElement('section');
      section.className = 'harness-section';
      const slug = node.textContent
        .toLowerCase()
        .replace(/[^a-z0-9]+/g, '-')
        .replace(/^-|-$/g, '');
      section.id = slug;
      content.append(section);
      const link = document.createElement('a');
      link.href = `#${slug}`;
      link.textContent = node.textContent;
      toc.append(link);
    }
    (section || content).append(node);
  });

  const progress = document.createElement('div');
  progress.className = 'harness-progress';
  progress.setAttribute('aria-hidden', 'true');
  document.body.append(progress);
  const updateProgress = () => {
    const scrollable = document.documentElement.scrollHeight - innerHeight;
    progress.style.width = `${scrollable > 0 ? (scrollY / scrollable) * 100 : 0}%`;
  };
  addEventListener('scroll', updateProgress, { passive: true });
  addEventListener('resize', updateProgress);
  updateProgress();
})();
