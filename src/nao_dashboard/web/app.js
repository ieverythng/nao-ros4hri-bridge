const POLL_MS = 1200
const TIMELINE_LIMIT = 160

const refs = {
  runMeta: document.getElementById('run-meta'),
  connection: document.getElementById('connection-pill'),
  updatedAt: document.getElementById('updated-at'),
  kpiGrid: document.getElementById('kpi-grid'),
  timeline: document.getElementById('timeline'),
  timelineCount: document.getElementById('timeline-count'),
  actionHealth: document.getElementById('action-health'),
  actionHealthMeta: document.getElementById('action-health-meta'),
  rosGraph: document.getElementById('ros-graph'),
  rosMeta: document.getElementById('ros-meta'),
  abRegistry: document.getElementById('ab-registry'),
  abMeta: document.getElementById('ab-meta'),
  timelineTemplate: document.getElementById('timeline-item-template'),
}

let lastState = null

async function loadState() {
  try {
    const response = await fetch('/api/state', { cache: 'no-store' })
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`)
    }
    const state = await response.json()
    lastState = state
    setOnline(true)
    render(state)
  } catch (err) {
    setOnline(false)
    if (lastState === null) {
      refs.runMeta.textContent = `Waiting for runtime (${err})`
    }
  }
}

function setOnline(isOnline) {
  refs.connection.className = `pill ${isOnline ? 'is-online' : 'is-offline'}`
  refs.connection.textContent = isOnline ? 'online' : 'offline'
}

function render(state) {
  const stats = state.stats || {}
  const events = (state.events || []).slice(-TIMELINE_LIMIT)
  const actionRows = state.action_health || []
  const rosGraph = state.ros_graph || {}
  const abRegistry = state.ab_registry || {}

  refs.runMeta.textContent = `${state.run_id || 'run_unknown'} | ${events.length} timeline events`
  refs.updatedAt.textContent = `updated ${formatTime(state.updated_at)}`

  renderKpis(stats)
  renderTimeline(events)
  renderActionHealth(actionRows)
  renderRosGraph(rosGraph)
  renderAbRegistry(abRegistry)
}

function renderKpis(stats) {
  const cards = [
    ['events', stats.event_count || 0],
    ['nodes', stats.node_count || 0],
    ['topics', stats.topic_count || 0],
    ['actions', stats.action_count || 0],
    ['AB objects', stats.ab_object_count || 0],
  ]

  refs.kpiGrid.innerHTML = cards
    .map(
      ([label, value]) =>
        `<article class="kpi-card"><p class="label">${escapeHtml(label)}</p><p class="value">${escapeHtml(
          String(value)
        )}</p></article>`
    )
    .join('')
}

function renderTimeline(events) {
  refs.timelineCount.textContent = `${events.length} events`
  refs.timeline.innerHTML = ''

  if (events.length === 0) {
    refs.timeline.innerHTML = '<p class="warning-line">No events yet. Start an interaction to populate timeline flow.</p>'
    return
  }

  for (const event of [...events].reverse()) {
    const node = refs.timelineTemplate.content.firstElementChild.cloneNode(true)
    node.querySelector('.event-type').textContent = event.event_type || 'message'
    node.querySelector('.channel').textContent = event.channel || ''
    node.querySelector('.timestamp').textContent = formatTime(event.timestamp)
    node.querySelector('.summary').textContent = event.payload_summary || ''
    refs.timeline.appendChild(node)
  }
}

function renderActionHealth(rows) {
  const onlineCount = rows.filter((item) => item.available).length
  refs.actionHealthMeta.textContent = `${onlineCount}/${rows.length} online`

  if (!rows.length) {
    refs.actionHealth.innerHTML = '<p class="warning-line">No action servers discovered yet.</p>'
    return
  }

  const tableRows = rows
    .map((row) => {
      const className = row.available ? 'status-ok' : 'status-missing'
      return `<tr><td>${escapeHtml(row.action_name || '')}</td><td class="${className}">${escapeHtml(
        row.status || ''
      )}</td></tr>`
    })
    .join('')

  refs.actionHealth.innerHTML = `<table><thead><tr><th>Action</th><th>Status</th></tr></thead><tbody>${tableRows}</tbody></table>`
}

function renderRosGraph(rosGraph) {
  const nodes = rosGraph.nodes || []
  const topics = rosGraph.topics || []
  const services = rosGraph.services || []
  const actions = rosGraph.actions || []

  refs.rosMeta.textContent = `${nodes.length} nodes | ${topics.length} topics`

  const rows = [
    ['nodes', nodes.length],
    ['topics', topics.length],
    ['services', services.length],
    ['actions', actions.length],
  ]
    .map(([label, value]) => `<tr><td>${escapeHtml(label)}</td><td>${escapeHtml(String(value))}</td></tr>`)
    .join('')

  const preview = actions
    .slice(0, 8)
    .map((row) => `<tr><td>${escapeHtml(row.name || '')}</td><td>${escapeHtml((row.types || []).join(', '))}</td></tr>`)
    .join('')

  refs.rosGraph.innerHTML = [
    `<table><thead><tr><th>Metric</th><th>Value</th></tr></thead><tbody>${rows}</tbody></table>`,
    '<div style="height:0.5rem"></div>',
    `<table><thead><tr><th>Discovered actions</th><th>Type</th></tr></thead><tbody>${preview || '<tr><td colspan="2">No actions yet.</td></tr>'}</tbody></table>`,
  ].join('')
}

function renderAbRegistry(abRegistry) {
  const objects = abRegistry.objects || []
  const edges = abRegistry.edges || []
  const errors = abRegistry.validation_errors || []

  refs.abMeta.textContent = `${objects.length} objects | ${edges.length} edges`

  const topRows = objects
    .slice(0, 14)
    .map((obj) => {
      const objectId = obj.object_id || obj.name || ''
      const level = obj.ab_level ?? '?'
      const mapping = obj.robot_adapter_mapping || obj.kind || ''
      return `<tr><td>${escapeHtml(String(objectId))}</td><td>${escapeHtml(String(level))}</td><td>${escapeHtml(
        String(mapping)
      )}</td></tr>`
    })
    .join('')

  const table = `<table><thead><tr><th>AB object</th><th>Level</th><th>Adapter</th></tr></thead><tbody>${
    topRows || '<tr><td colspan="3">AB registry is empty.</td></tr>'
  }</tbody></table>`

  const errorLines = errors.map((text) => `<p class="warning-line">${escapeHtml(text)}</p>`).join('')
  refs.abRegistry.innerHTML = `${errorLines}${table}`
}

function escapeHtml(text) {
  return String(text)
    .replaceAll('&', '&amp;')
    .replaceAll('<', '&lt;')
    .replaceAll('>', '&gt;')
    .replaceAll('"', '&quot;')
    .replaceAll("'", '&#39;')
}

function formatTime(unixSeconds) {
  if (!unixSeconds) {
    return 'n/a'
  }
  const date = new Date(Number(unixSeconds) * 1000)
  if (Number.isNaN(date.getTime())) {
    return 'n/a'
  }
  return date.toLocaleTimeString([], { hour12: false })
}

loadState()
setInterval(loadState, POLL_MS)
