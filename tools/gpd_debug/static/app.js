// GPD Debug Visualizer — Plotly 3D + UI logic

// ── Gripper geometry constants ───────────────────────────────────────
const FINGER_WIDTH = 0.01;
const HAND_DEPTH = 0.028;
const HAND_OUTER_DIAMETER = 0.08;

// ── State ────────────────────────────────────────────────────────────
let currentGrasps = [];
let currentMeta = {};
let graspTraceIndices = {};  // maps grasp index -> plotly trace indices

// ── Color scheme ─────────────────────────────────────────────────────
function graspCategory(g) {
  // Determine category based on filter results
  if (g.filter_z && !g.filter_z.pass) return 'filter_fail';
  if (g.filter_cone && !g.filter_cone.pass) return 'filter_fail';
  if (g.filter_orient && !g.filter_orient.pass) return 'orient_fail';
  // Passed filters — check IK
  const ikPassed = (g.ik_6dof && g.ik_6dof.pass) ||
                   (g.ik_topdown && g.ik_topdown.pass) ||
                   (g.ik_posonly && g.ik_posonly.pass);
  if (!ikPassed && (g.ik_6dof || g.ik_topdown || g.ik_posonly)) return 'ik_fail';
  if (g.selected) return 'passed';
  return 'passed';  // passed filters, IK not attempted yet or succeeded
}

const CATEGORY_COLORS = {
  passed: '#00ff88',
  ik_fail: '#ffdd44',
  orient_fail: '#ff8800',
  filter_fail: '#ff3344',
};

const CATEGORY_LABELS = {
  passed: 'Passed',
  ik_fail: 'IK Failed',
  orient_fail: 'Orient Failed',
  filter_fail: 'Z/Cone Failed',
};

// ── Gripper wireframe generation ─────────────────────────────────────

function buildGripperTrace(g, color, opacity) {
  // Build a simplified gripper wireframe for a single grasp
  const p = g.position;
  const a = g.approach;   // approach direction
  const ax = g.axis;      // finger opening direction
  const w = g.width || 0.04;

  // Wrist point (back along approach)
  const wrist = [
    p[0] - HAND_DEPTH * a[0],
    p[1] - HAND_DEPTH * a[1],
    p[2] - HAND_DEPTH * a[2],
  ];

  // Finger offset along axis
  const halfW = w / 2 + FINGER_WIDTH / 2;

  // Left finger: wrist side -> tip side (along approach), offset +halfW along axis
  const lBase = [
    wrist[0] + halfW * ax[0],
    wrist[1] + halfW * ax[1],
    wrist[2] + halfW * ax[2],
  ];
  const lTip = [
    p[0] + halfW * ax[0],
    p[1] + halfW * ax[1],
    p[2] + halfW * ax[2],
  ];
  const lInner = [
    p[0] + (w / 2) * ax[0],
    p[1] + (w / 2) * ax[1],
    p[2] + (w / 2) * ax[2],
  ];

  // Right finger
  const rBase = [
    wrist[0] - halfW * ax[0],
    wrist[1] - halfW * ax[1],
    wrist[2] - halfW * ax[2],
  ];
  const rTip = [
    p[0] - halfW * ax[0],
    p[1] - halfW * ax[1],
    p[2] - halfW * ax[2],
  ];
  const rInner = [
    p[0] - (w / 2) * ax[0],
    p[1] - (w / 2) * ax[1],
    p[2] - (w / 2) * ax[2],
  ];

  // Build trace: wrist bar, left finger U, right finger U
  // Path: lBase -> lTip -> lInner -> null -> rInner -> rTip -> rBase -> null -> lBase -> rBase
  const pts = [
    lBase, lTip, lInner, null, rInner, rTip, rBase, null, lBase, rBase
  ];

  const x = [], y = [], z = [];
  for (const pt of pts) {
    if (pt === null) {
      x.push(null); y.push(null); z.push(null);
    } else {
      x.push(pt[0]); y.push(pt[1]); z.push(pt[2]);
    }
  }

  return {
    type: 'scatter3d',
    mode: 'lines',
    x, y, z,
    line: { color: color, width: 3 },
    opacity: opacity,
    hoverinfo: 'text',
    hovertext: `Grasp #${g.index} (score: ${g.score.toFixed(1)})`,
    showlegend: false,
  };
}

function buildApproachArrow(g, color) {
  const p = g.position;
  const a = g.approach;
  const len = 0.04;  // 4cm arrow
  const tip = [
    p[0] + len * a[0],
    p[1] + len * a[1],
    p[2] + len * a[2],
  ];
  return {
    type: 'scatter3d',
    mode: 'lines+markers',
    x: [p[0], tip[0]],
    y: [p[1], tip[1]],
    z: [p[2], tip[2]],
    line: { color: color, width: 2 },
    marker: { size: [3, 1], color: color },
    hoverinfo: 'skip',
    showlegend: false,
  };
}

// ── Plot building ────────────────────────────────────────────────────

function buildScene(pointcloud, grasps, meta) {
  const traces = [];

  // Scene cloud (gray)
  if (pointcloud.scene) {
    traces.push({
      type: 'scatter3d',
      mode: 'markers',
      x: pointcloud.scene.x,
      y: pointcloud.scene.y,
      z: pointcloud.scene.z,
      marker: { size: 1, color: '#666', opacity: 0.3 },
      hoverinfo: 'skip',
      name: 'Scene',
      showlegend: true,
    });
  }

  // Object cloud (blue)
  if (pointcloud.object) {
    traces.push({
      type: 'scatter3d',
      mode: 'markers',
      x: pointcloud.object.x,
      y: pointcloud.object.y,
      z: pointcloud.object.z,
      marker: { size: 2, color: '#4488ff', opacity: 0.7 },
      hoverinfo: 'skip',
      name: 'Object crop',
      showlegend: true,
    });
  }

  // Origin axes (base_link frame reference)
  const axLen = 0.05;
  traces.push({
    type: 'scatter3d', mode: 'lines',
    x: [0, axLen, null, 0, 0, null, 0, 0],
    y: [0, 0, null, 0, axLen, null, 0, 0],
    z: [0, 0, null, 0, 0, null, 0, axLen],
    line: { color: ['red','red',null,'green','green',null,'blue','blue'], width: 3 },
    hoverinfo: 'skip', showlegend: false,
  });

  // Object position marker
  if (meta.object_pos) {
    traces.push({
      type: 'scatter3d', mode: 'markers',
      x: [meta.object_pos[0]],
      y: [meta.object_pos[1]],
      z: [meta.object_pos[2]],
      marker: { size: 8, color: '#00ff88', symbol: 'diamond' },
      hovertext: `Object: ${meta.target || '?'}`,
      hoverinfo: 'text',
      name: 'Object',
      showlegend: true,
    });
  }

  // Grasp wireframes
  graspTraceIndices = {};
  const baseIdx = traces.length;
  for (let i = 0; i < grasps.length; i++) {
    const g = grasps[i];
    const cat = graspCategory(g);
    const color = CATEGORY_COLORS[cat];
    const opacity = g.selected ? 1.0 : 0.6;

    graspTraceIndices[g.index] = [traces.length, traces.length + 1];
    traces.push(buildGripperTrace(g, color, opacity));
    traces.push(buildApproachArrow(g, color));
  }

  return traces;
}

function plotScene(traces) {
  const layout = {
    scene: {
      aspectmode: 'data',
      xaxis: { title: 'X (m)', color: '#888', gridcolor: '#333' },
      yaxis: { title: 'Y (m)', color: '#888', gridcolor: '#333' },
      zaxis: { title: 'Z (m)', color: '#888', gridcolor: '#333' },
      bgcolor: '#1a1a2e',
    },
    paper_bgcolor: '#1a1a2e',
    plot_bgcolor: '#1a1a2e',
    font: { color: '#e0e0e0' },
    legend: {
      x: 0.01, y: 0.99,
      bgcolor: 'rgba(22,33,62,0.8)',
      font: { size: 11 },
    },
    margin: { l: 0, r: 0, t: 0, b: 0 },
  };

  Plotly.newPlot('plot3d', traces, layout, { responsive: true });
}

// ── Grasp list UI ────────────────────────────────────────────────────

function buildGraspList(grasps) {
  const container = document.getElementById('grasp-list');
  container.innerHTML = '';

  for (const g of grasps) {
    const cat = graspCategory(g);
    const color = CATEGORY_COLORS[cat];

    const item = document.createElement('div');
    item.className = 'grasp-item';
    item.dataset.index = g.index;

    const dot = document.createElement('span');
    dot.className = 'grasp-dot';
    dot.style.background = color;

    const label = document.createElement('span');
    const selectedTag = g.selected ? ' [EXEC]' : '';
    label.textContent = `#${g.index} score=${g.score.toFixed(1)} w=${(g.width*1000).toFixed(0)}mm${selectedTag}`;

    item.appendChild(dot);
    item.appendChild(label);
    item.addEventListener('click', () => selectGrasp(g));
    container.appendChild(item);
  }
}

function selectGrasp(g) {
  // Highlight in list
  document.querySelectorAll('.grasp-item').forEach(el => {
    el.classList.toggle('selected', parseInt(el.dataset.index) === g.index);
  });

  // Show details
  const detail = document.getElementById('grasp-detail');
  const cat = graspCategory(g);
  let text = `Grasp #${g.index}\n`;
  text += `Category: ${CATEGORY_LABELS[cat]}\n`;
  text += `Score: ${g.score.toFixed(3)}\n`;
  text += `Width: ${(g.width * 1000).toFixed(1)}mm\n`;
  text += `Position: (${g.position.map(v => v.toFixed(3)).join(', ')})\n`;
  text += `Approach: (${g.approach.map(v => v.toFixed(2)).join(', ')})\n`;
  text += `Axis:     (${g.axis.map(v => v.toFixed(2)).join(', ')})\n`;
  text += `\n── Filters ──\n`;
  if (g.filter_z) text += `Z filter: ${g.filter_z.pass ? 'PASS' : 'FAIL'} (z=${g.filter_z.value.toFixed(3)})\n`;
  if (g.filter_cone) text += `Cone filter: ${g.filter_cone.pass ? 'PASS' : 'FAIL'} (az=${g.filter_cone.approach_z.toFixed(2)})\n`;
  if (g.filter_orient) {
    const al = g.filter_orient.alignment !== null ? g.filter_orient.alignment.toFixed(2) : 'N/A';
    text += `Orient filter: ${g.filter_orient.pass ? 'PASS' : 'FAIL'} (align=${al})\n`;
  }
  text += `\n── IK ──\n`;
  if (g.ik_6dof) text += `6-DOF: ${g.ik_6dof.pass ? 'PASS' : 'FAIL'} ${g.ik_6dof.msg}\n`;
  if (g.ik_topdown) text += `Top-down: ${g.ik_topdown.pass ? 'PASS' : 'FAIL'} ${g.ik_topdown.msg}\n`;
  if (g.ik_posonly) text += `Pos-only: ${g.ik_posonly.pass ? 'PASS' : 'FAIL'} ${g.ik_posonly.msg}\n`;
  if (g.selected) text += `\n** SELECTED FOR EXECUTION **`;
  detail.textContent = text;
}

// ── Filter toggle logic ──────────────────────────────────────────────

function applyFilters() {
  const showPassed = document.getElementById('show-passed').checked;
  const showIkFail = document.getElementById('show-ik-fail').checked;
  const showOrientFail = document.getElementById('show-orient-fail').checked;
  const showFilterFail = document.getElementById('show-filter-fail').checked;

  const updates = [];
  for (const g of currentGrasps) {
    const cat = graspCategory(g);
    const traceIdxs = graspTraceIndices[g.index];
    if (!traceIdxs) continue;

    let visible;
    switch (cat) {
      case 'passed': visible = showPassed; break;
      case 'ik_fail': visible = showIkFail; break;
      case 'orient_fail': visible = showOrientFail; break;
      case 'filter_fail': visible = showFilterFail; break;
      default: visible = true;
    }

    for (const idx of traceIdxs) {
      updates.push({ traceIdx: idx, visible });
    }
  }

  // Batch update visibility
  const traceIndices = updates.map(u => u.traceIdx);
  const visibilities = updates.map(u => u.visible);
  if (traceIndices.length > 0) {
    // Plotly restyle expects one update per trace
    for (let i = 0; i < traceIndices.length; i++) {
      Plotly.restyle('plot3d', { visible: visibilities[i] }, [traceIndices[i]]);
    }
  }
}

// ── Data loading ─────────────────────────────────────────────────────

async function loadSessions() {
  const resp = await fetch('/api/sessions');
  const sessions = await resp.json();
  const sel = document.getElementById('session-select');
  sel.innerHTML = '';
  for (const s of sessions) {
    const opt = document.createElement('option');
    opt.value = s;
    opt.textContent = s.replace('session_', '');
    sel.appendChild(opt);
  }
  if (sessions.length > 0) {
    await loadCaptures(sessions[0]);
  }
}

async function loadCaptures(session) {
  const resp = await fetch(`/api/session/${session}`);
  const data = await resp.json();
  const sel = document.getElementById('capture-select');
  sel.innerHTML = '';
  for (const c of data.captures) {
    const opt = document.createElement('option');
    opt.value = c;
    opt.textContent = c.replace('capture_', '#');
    sel.appendChild(opt);
  }
}

async function loadCapture() {
  const session = document.getElementById('session-select').value;
  const capture = document.getElementById('capture-select').value;
  if (!session || !capture) return;

  // Fetch data in parallel
  const [captureResp, pcResp] = await Promise.all([
    fetch(`/api/capture/${session}/${capture}`),
    fetch(`/api/pointcloud/${session}/${capture}`),
  ]);

  const captureData = await captureResp.json();
  const pointcloud = await pcResp.json();

  currentGrasps = captureData.grasps || [];
  currentMeta = captureData.meta || {};

  // Build and render 3D scene
  const traces = buildScene(pointcloud, currentGrasps, currentMeta);
  plotScene(traces);

  // Build grasp list
  buildGraspList(currentGrasps);

  // Update images
  document.getElementById('rgb-img').src = `/api/rgb/${session}/${capture}`;
  document.getElementById('depth-img').src = `/api/depth/${session}/${capture}`;

  // Reset detail
  document.getElementById('grasp-detail').textContent =
    `Loaded ${currentGrasps.length} grasps\nTarget: ${currentMeta.target || '?'}`;
}

// ── Event listeners ──────────────────────────────────────────────────

document.getElementById('session-select').addEventListener('change', async (e) => {
  await loadCaptures(e.target.value);
});

document.getElementById('load-btn').addEventListener('click', loadCapture);

document.getElementById('show-passed').addEventListener('change', applyFilters);
document.getElementById('show-ik-fail').addEventListener('change', applyFilters);
document.getElementById('show-orient-fail').addEventListener('change', applyFilters);
document.getElementById('show-filter-fail').addEventListener('change', applyFilters);

// Auto-load on page load
loadSessions().then(() => {
  const capSel = document.getElementById('capture-select');
  if (capSel.options.length > 0) {
    loadCapture();
  }
});
