#pragma once

// dashboard page -- lives in flash (PROGMEM), served as-is. History/graphing is done client
// side (browser localStorage), the ESP32 only ever hands out the current live reading via
// /data; see the header comment in the .ino for why (keeps this sketch from needing an
// in-RAM history buffer, at the cost of history not surviving a page reload on a fresh
// browser/device).
//
// This lives in its own header rather than inline in the .ino on purpose: Arduino's
// auto-prototype-generation pass (ctags-based) only scans .ino files, and it doesn't
// understand raw string literals -- JS in here that looks like a C++ function definition
// (e.g. "function poll() {") fools it into splitting the string apart mid-file, which then
// fails to compile as top-level C++. Keeping this in a .h that's #included sidesteps that
// scan entirely.
const char dashboardHtml[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Spool2Spool</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<style>
  body { font-family: sans-serif; background:#fff; color:#222; margin:0; padding:16px; }
  h1 { font-size:1.2em; margin:0 0 12px; }
  table { border-collapse:collapse; margin-bottom:16px; }
  td { padding:4px 12px 4px 0; }
  .val { font-weight:bold; }
  .btn {
    display:inline-block; padding:8px 16px; background:#0a6cff; color:#fff;
    text-decoration:none; border-radius:4px; font-weight:bold;
  }
  .btn:hover { background:#0857cc; }
  .btn.secondary { background:#888; }
  .btn.secondary:hover { background:#666; }
</style>
</head>
<body>
<h1>Spool2Spool</h1>
<table>
  <tr><td>Speed</td><td class="val" id="speed">--</td><td>Target</td><td class="val" id="targetSpeed">--</td></tr>
  <tr><td>Angle</td><td class="val" id="angle">--</td><td>Target</td><td class="val" id="targetAngle">--</td></tr>
  <tr><td>M1</td><td class="val" id="m1">--</td><td>M2</td><td class="val" id="m2">--</td></tr>
</table>
<canvas id="chart" height="120"></canvas>
<p>
  <a class="btn" href="/update">Firmware update (OTA)</a>
  <button class="btn secondary" id="purgeBtn" type="button">Clear graph history</button>
</p>
<script>
const HISTORY_KEY = 's2sHistory';
const MAX_POINTS = 1800; // ~30min at 1 sample/sec, capped so localStorage doesn't grow unbounded
let history = [];
try {
  history = JSON.parse(localStorage.getItem(HISTORY_KEY)) || [];
} catch (e) {
  history = [];
}

const ctx = document.getElementById('chart').getContext('2d');
const chart = new Chart(ctx, {
  type: 'line',
  data: {
    labels: history.map(p => new Date(p.t).toLocaleTimeString()),
    datasets: [
      { label: 'Speed (m/min)', data: history.map(p => p.speed), borderColor: '#0a6cff', borderWidth: 1.5, pointRadius: 0, tension: 0.2 },
      { label: 'Speed setpoint', data: history.map(p => p.targetSpeed), borderColor: '#0a6cff', borderWidth: 1.5, borderDash: [4, 3], pointRadius: 0, tension: 0.2 },
      { label: 'Angle (deg)', data: history.map(p => p.angle), borderColor: '#e08a00', borderWidth: 1.5, pointRadius: 0, tension: 0.2, yAxisID: 'y1' },
      { label: 'Angle setpoint', data: history.map(p => p.targetAngle), borderColor: '#e08a00', borderWidth: 1.5, borderDash: [4, 3], pointRadius: 0, tension: 0.2, yAxisID: 'y1' }
    ]
  },
  options: {
    animation: false,
    scales: {
      y: { title: { display: true, text: 'm/min' } },
      y1: { position: 'right', title: { display: true, text: 'deg' }, grid: { drawOnChartArea: false } }
    }
  }
});

function poll() {
  fetch('/data').then(r => r.json()).then(d => {
    document.getElementById('speed').textContent = d.speed.toFixed(1) + ' m/min';
    document.getElementById('targetSpeed').textContent = d.targetSpeed.toFixed(1) + ' m/min';
    document.getElementById('angle').textContent = d.angle.toFixed(1) + ' deg';
    document.getElementById('targetAngle').textContent = d.targetAngle.toFixed(1) + ' deg';
    document.getElementById('m1').textContent = d.mode1 + ' ' + d.m1Percent + '%';
    document.getElementById('m2').textContent = d.mode2 + ' ' + d.m2Percent + '%';

    history.push({ t: Date.now(), speed: d.speed, targetSpeed: d.targetSpeed, angle: d.angle, targetAngle: d.targetAngle });
    if (history.length > MAX_POINTS) history.shift();
    try { localStorage.setItem(HISTORY_KEY, JSON.stringify(history)); } catch (e) {}

    updateChartData();
  }).catch(() => {});
}

function updateChartData() {
  chart.data.labels = history.map(p => new Date(p.t).toLocaleTimeString());
  chart.data.datasets[0].data = history.map(p => p.speed);
  chart.data.datasets[1].data = history.map(p => p.targetSpeed);
  chart.data.datasets[2].data = history.map(p => p.angle);
  chart.data.datasets[3].data = history.map(p => p.targetAngle);
  chart.update('none');
}

document.getElementById('purgeBtn').addEventListener('click', () => {
  history = [];
  try { localStorage.removeItem(HISTORY_KEY); } catch (e) {}
  updateChartData();
});

setInterval(poll, 1000);
poll();
</script>
</body>
</html>
)rawliteral";
