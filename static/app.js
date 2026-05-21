const FIX_LABELS = { 0: "None", 1: "GPS", 2: "DGPS", 4: "RTK Fixed", 5: "RTK Float" };

// ── WebSocket ──────────────────────────────────────────────────────────────
let ws = null;

function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);

  ws.onopen = () => setStatus(true);

  ws.onmessage = (ev) => {
    const data = JSON.parse(ev.data);
    updateDiagnostics(data.diagnostics || {});
    updateGps(data.gps || {});
    updateNodes(data.nodes || {});
    updateWeather(data.weather || {});
  };

  ws.onclose = () => {
    setStatus(false);
    setTimeout(connect, 3000);
  };

  ws.onerror = () => ws.close();
}

function setStatus(online) {
  document.getElementById("ws-dot").className = "dot " + (online ? "online" : "offline");
  document.getElementById("ws-label").textContent = online ? "connected" : "reconnecting";
}

// ── Clock ──────────────────────────────────────────────────────────────────
function tickClock() {
  const now = new Date();
  document.getElementById("clock").textContent =
    now.toLocaleTimeString("en-GB", { hour12: false });
}
setInterval(tickClock, 1000);
tickClock();

// ── Diagnostics ────────────────────────────────────────────────────────────
function updateDiagnostics(d) {
  setText("d-cpu",    d.cpu_percent != null ? d.cpu_percent.toFixed(1) + "%" : "--");
  setText("d-temp",   d.cpu_temp   != null ? d.cpu_temp + "°C" : "N/A");
  setText("d-mem",    d.mem_percent != null ? d.mem_percent + "%" : "--");
  setText("d-load",   d.load_1     != null ? d.load_1.toFixed(2) : "--");
  setText("d-uptime", d.uptime_s   != null ? formatUptime(d.uptime_s) : "--");

  if (d.mem_percent != null) {
    const bar = document.getElementById("mem-bar");
    bar.style.width = d.mem_percent + "%";
    bar.style.background = d.mem_percent > 85 ? "var(--red)"
                         : d.mem_percent > 65 ? "var(--yellow)"
                         : "var(--accent)";
  }
}

function formatUptime(s) {
  const d = Math.floor(s / 86400);
  const h = Math.floor((s % 86400) / 3600);
  const m = Math.floor((s % 3600) / 60);
  return d > 0 ? `${d}d ${h}h ${m}m` : h > 0 ? `${h}h ${m}m` : `${m}m`;
}

// ── GPS ────────────────────────────────────────────────────────────────────
function updateGps(g) {
  const err = document.getElementById("g-error");
  if (g.error) {
    err.textContent = g.error;
    err.classList.remove("hidden");
    return;
  }
  err.classList.add("hidden");
  setText("g-lat", g.lat  != null ? g.lat.toFixed(6)  : "--");
  setText("g-lon", g.lon  != null ? g.lon.toFixed(6)  : "--");
  setText("g-alt", g.altitude != null ? g.altitude + " m" : "--");
  setText("g-spd", g.speed_kmh != null ? g.speed_kmh + " km/h" : "--");
  setText("g-sat", g.satellites ?? "--");
  setText("g-fix", FIX_LABELS[g.fix] ?? "--");
}

// ── Nodes ──────────────────────────────────────────────────────────────────
const _nodeOrder = [];

function updateNodes(nodes) {
  const grid = document.getElementById("node-grid");
  const noNodes = document.getElementById("no-nodes");
  const ids = Object.keys(nodes);

  noNodes.style.display = ids.length ? "none" : "block";
  document.getElementById("node-count").textContent =
    ids.length + (ids.length === 1 ? " node" : " nodes");

  // create cards for new nodes
  for (const id of ids) {
    if (!_nodeOrder.includes(id)) {
      _nodeOrder.push(id);
      const card = document.createElement("div");
      card.className = "node-card";
      card.id = "node-" + id;
      grid.appendChild(card);
    }
    renderNode(id, nodes[id]);
  }
}

function renderNode(id, data) {
  const card = document.getElementById("node-" + id);
  if (!card) return;

  const age = data._ts ? (Date.now() / 1000 - data._ts) : null;
  card.className = "node-card " + (age == null ? "" : age < 10 ? "fresh" : age < 60 ? "" : age < 300 ? "stale" : "dead");

  const fields = Object.entries(data).filter(([k]) => !k.startsWith("_"));

  card.innerHTML = `
    <div class="node-header">
      <span class="node-id">${escHtml(id)}</span>
      <span class="node-age">${age != null ? formatAge(age) : ""}</span>
    </div>
    <div class="node-fields">
      ${fields.map(([k, v]) => `
        <div class="node-field">
          <span class="field-key">${escHtml(k)}</span>
          <span class="field-val">${escHtml(formatVal(v))}</span>
        </div>`).join("")}
    </div>`;
}

function formatAge(s) {
  if (s < 60) return Math.round(s) + "s ago";
  if (s < 3600) return Math.round(s / 60) + "m ago";
  return Math.round(s / 3600) + "h ago";
}

function formatVal(v) {
  if (v === null || v === undefined) return "--";
  if (typeof v === "number") return Number.isInteger(v) ? String(v) : v.toFixed(2);
  return String(v);
}

// ── Weather ────────────────────────────────────────────────────────────────
function updateWeather(w) {
  const err = document.getElementById("w-error");
  if (w.error && !w.temp) {
    err.textContent = w.error;
    err.classList.remove("hidden");
  } else {
    err.classList.add("hidden");
  }

  setText("w-temp", w.temp != null ? w.temp + "°C" : "--");
  setText("w-cond", w.condition || "--");
  setText("w-wind", w.wind_kmh != null ? w.wind_kmh + " km/h" : "--");
  setText("w-hum",  w.humidity != null ? w.humidity + "%" : "--");
  setText("w-rise", w.sunrise ? fmtTime(w.sunrise) : "--");
  setText("w-set",  w.sunset  ? fmtTime(w.sunset)  : "--");

  const fc = document.getElementById("forecast");
  if (w.forecast && w.forecast.length) {
    fc.innerHTML = w.forecast.map(f => `
      <div class="forecast-row">
        <span class="forecast-date">${fmtDate(f.date)}</span>
        <span class="forecast-cond">${escHtml(f.condition)}</span>
        <span class="forecast-range">${f.max}° / ${f.min}°</span>
      </div>`).join("");
  }
}

// Open-Meteo returns local time strings without a TZ suffix ("2026-05-21T06:03").
// Passing those to new Date() causes two distinct bugs:
//   - datetime strings → parsed as browser local time (wrong when browser ≠ location TZ)
//   - date-only strings → parsed as UTC midnight (rolls back one day in UTC-3)
// So we extract the parts directly instead of delegating to Date.

function fmtTime(iso) {
  if (typeof iso === "string" && iso.includes("T")) return iso.split("T")[1].slice(0, 5);
  const d = new Date(iso);
  return isNaN(d) ? iso : d.toLocaleTimeString("en-GB", { hour: "2-digit", minute: "2-digit", hour12: false });
}

function fmtDate(iso) {
  if (typeof iso === "string" && /^\d{4}-\d{2}-\d{2}/.test(iso)) {
    const [y, m, d] = iso.slice(0, 10).split("-").map(Number);
    return new Date(y, m - 1, d).toLocaleDateString("en-GB", { weekday: "short", day: "numeric", month: "short" });
  }
  const d = new Date(iso);
  return isNaN(d) ? iso : d.toLocaleDateString("en-GB", { weekday: "short", day: "numeric", month: "short" });
}

// ── Helpers ────────────────────────────────────────────────────────────────
function setText(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}

function escHtml(s) {
  return String(s)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;");
}

connect();
