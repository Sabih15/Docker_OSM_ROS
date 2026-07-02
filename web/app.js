/* RR100 · OvGU Ground Control — WebSocket → Leaflet map client.
 *
 * Receives JSON state frames from the ROS 2 ws_bridge (~30 Hz) and renders:
 *   - a live amber marker at the robot's lat/lon, with a fading comet trail
 *   - linear / angular velocity readouts
 *   - the measured stream rate (frames per second)
 *   - a bearing rose derived from successive positions
 */

// ----- Config ---------------------------------------------------------------
const OVGU = [52.139200, 11.645200];   // map centre: OvGU Universitaetsplatz
const WS_PORT = 9090;
const WS_URL = `ws://${location.hostname || "localhost"}:${WS_PORT}`;
const TRAIL_MAX = 300;                 // points retained in the trail
const STALE_MS = 1500;                 // no frames for this long ⇒ "lost"

// ----- Map ------------------------------------------------------------------
const map = L.map("map", { zoomControl: true, attributionControl: true })
  .setView(OVGU, 18);

L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
  maxZoom: 19,
  attribution: '&copy; OpenStreetMap contributors',
}).addTo(map);

// A small ring marking the configured campus origin.
L.circleMarker(OVGU, {
  radius: 6, color: "#4fd1c5", weight: 1.5, fill: false, opacity: 0.8,
}).addTo(map).bindTooltip("OvGU origin", { permanent: false });

const trail = L.polyline([], {
  color: "#ffb454", weight: 3, opacity: 0.55, lineJoin: "round",
}).addTo(map);

const robotIcon = L.divIcon({
  className: "robot-marker",
  html: '<div class="robot-marker__pulse"></div><div class="robot-marker__core"></div>',
  iconSize: [14, 14],
  iconAnchor: [7, 7],
});
let robot = null;          // created on first valid fix
let firstFix = true;

// ----- DOM ------------------------------------------------------------------
const el = {
  link:    document.getElementById("link"),
  linkLbl: document.querySelector("#link .link__label"),
  lat:     document.getElementById("lat"),
  lon:     document.getElementById("lon"),
  linear:  document.getElementById("linear"),
  angular: document.getElementById("angular"),
  rate:    document.getElementById("rate"),
  needle:  document.getElementById("needle"),
  bearing: document.getElementById("bearing"),
};

// Build the rose tick marks (every 30°).
(function buildRoseTicks() {
  const g = document.getElementById("roseTicks");
  for (let a = 0; a < 360; a += 30) {
    const rad = (a - 90) * Math.PI / 180;
    const x1 = 60 + Math.cos(rad) * 50, y1 = 64 + Math.sin(rad) * 50;
    const x2 = 60 + Math.cos(rad) * 44, y2 = 64 + Math.sin(rad) * 44;
    const ln = document.createElementNS("http://www.w3.org/2000/svg", "line");
    ln.setAttribute("x1", x1); ln.setAttribute("y1", y1);
    ln.setAttribute("x2", x2); ln.setAttribute("y2", y2);
    g.appendChild(ln);
  }
})();

function setLink(state, label) {
  el.link.className = `link link--${state}`;
  el.linkLbl.textContent = label;
}

// ----- Bearing from successive points (great-circle initial bearing) --------
let lastLatLng = null;
let displayBearing = 0;
function bearingDeg(a, b) {
  const φ1 = a[0] * Math.PI / 180, φ2 = b[0] * Math.PI / 180;
  const Δλ = (b[1] - a[1]) * Math.PI / 180;
  const y = Math.sin(Δλ) * Math.cos(φ2);
  const x = Math.cos(φ1) * Math.sin(φ2) - Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
  return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
}

// ----- Stream-rate meter (frames in a sliding 1 s window) -------------------
const frameTimes = [];
function measureRate(now) {
  frameTimes.push(now);
  while (frameTimes.length && now - frameTimes[0] > 1000) frameTimes.shift();
  return frameTimes.length;
}

// ----- Frame handling -------------------------------------------------------
let lastFrameAt = 0;

function onState(s) {
  const now = performance.now();
  lastFrameAt = now;

  // Velocities + rate always update.
  el.linear.textContent  = fmt(s.linear, 3);
  el.angular.textContent = fmt(s.angular, 3);
  el.rate.textContent    = measureRate(now);

  setLink("live", "Live");

  if (typeof s.lat !== "number" || typeof s.lon !== "number") return;
  const here = [s.lat, s.lon];

  el.lat.textContent = s.lat.toFixed(6);
  el.lon.textContent = s.lon.toFixed(6);

  if (!robot) {
    robot = L.marker(here, { icon: robotIcon, interactive: false }).addTo(map);
  } else {
    robot.setLatLng(here);
  }

  // Trail (cap length).
  const pts = trail.getLatLngs();
  pts.push(here);
  if (pts.length > TRAIL_MAX) pts.shift();
  trail.setLatLngs(pts);

  // Keep the robot in view: recentre on first fix, then gently pan if it drifts.
  if (firstFix) { map.setView(here, 18); firstFix = false; }
  else if (!map.getBounds().pad(-0.25).contains(here)) { map.panTo(here, { animate: true }); }

  // Bearing rose from movement.
  if (lastLatLng) {
    const moved = (here[0] - lastLatLng[0]) ** 2 + (here[1] - lastLatLng[1]) ** 2;
    if (moved > 1e-12) {              // ignore jitter when stationary
      displayBearing = bearingDeg(lastLatLng, here);
      el.needle.style.transform = `rotate(${displayBearing}deg)`;
      el.bearing.textContent = Math.round(displayBearing).toString().padStart(3, "0");
    }
  }
  lastLatLng = here;
}

function fmt(v, d) {
  return (typeof v === "number") ? v.toFixed(d) : "—";
}

// ----- WebSocket with auto-reconnect ---------------------------------------
let ws = null;
let backoff = 500;

function connect() {
  setLink("connecting", "Connecting…");
  ws = new WebSocket(WS_URL);

  ws.onopen = () => { backoff = 500; setLink("live", "Live"); };

  ws.onmessage = (ev) => {
    let frame;
    try { frame = JSON.parse(ev.data); } catch { return; }
    if (frame && frame.type === "state") onState(frame);
  };

  ws.onclose = () => {
    setLink("lost", "Link lost");
    setTimeout(connect, backoff);
    backoff = Math.min(backoff * 1.7, 5000);   // capped exponential backoff
  };

  ws.onerror = () => { try { ws.close(); } catch {} };
}

// Mark the link stale if frames stop arriving even while the socket is open.
setInterval(() => {
  if (lastFrameAt && performance.now() - lastFrameAt > STALE_MS &&
      el.link.classList.contains("link--live")) {
    setLink("lost", "No data");
    el.rate.textContent = "0";
  }
}, 500);

connect();
