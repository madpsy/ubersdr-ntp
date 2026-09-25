"use strict";
// ---------------------------------------------------------------------------
// The signal path, drawn: every transmitter and upstream server, the receivers
// and peers that hear them, this daemon, and what it serves -- NTP clients and
// the browser showing this page -- with each link coloured by whether it is
// feeding the served time, measured but standing by, or down.
//
// Nothing here is fetched. The page already holds everything on the event
// stream: the 1 Hz tick for states and offsets, the every-few-seconds status
// document for receiver names, carriers and the delay model, and the page's
// own /api/time measurement for the browser's link. index.html hands them over
// through window.Flow and this file only draws.
//
// Its own file because it is a self-contained drawing with its own styles, and
// the page is long enough. tools/embed-html.py inlines it into the page at
// build time, so the binary still serves one document.
// ---------------------------------------------------------------------------
(function () {
const NS = "http://www.w3.org/2000/svg";

// Transmitter sites, as the daemon's propagation model has them.
const STATIONS = {
  wwv:  { name: "WWV",  where: "Fort Collins, Colorado" },
  wwvh: { name: "WWVH", where: "Kekaha, Kauaʻi, Hawaii" },
  wwvb: { name: "WWVB", where: "Fort Collins, Colorado" },
  dcf77: { name: "DCF77", where: "Mainflingen, Germany" },
  msf:   { name: "MSF",   where: "Anthorn, Cumbria, UK" },
  allouis: { name: "Allouis", where: "Allouis, near Vierzon, France" },
  unknown: { name: "WWV / WWVH", where: "station not identified yet" },
};

// One colour per kind of hop, so a path reads by colour before any label is
// read: radio waves, the receiver's WebSocket, NTP over UDP, and what goes out.
const HUE = { rf: "#38bdf8", ws: "#22d3ee", ntp: "#a78bfa", out: "#34d399", bad: "#fb7185" };

const reduced = matchMedia("(prefers-reduced-motion: reduce)");

const esc = (s) => String(s == null ? "" : s)
  .replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
const fin = (v) => v != null && isFinite(v);
const num = (v, p = 1) => fin(v) ? v.toFixed(p) : "—";
const sgn = (v, p = 2) => fin(v) ? (v >= 0 ? "+" : "−") + Math.abs(v).toFixed(p) : "—";
const msv = (v, p = 2) => fin(v) ? num(v, p) + " ms" : "—";
const freq = (hz) => !fin(hz) || !hz ? "" : hz < 1e6 ? (hz / 1e3).toFixed(0) + " kHz"
  : (hz / 1e6).toFixed(hz % 1e6 ? 1 : 0) + " MHz";

let root = null, scroller, grid, svg, canvas, ctx, chipLayer, summary, detail;
// The node whose full box is shown under the diagram in its compact form.
let selected = "core", fullHtml = {}, kindOf = {};
let tick = null, status = {}, browser = null, linkOk = false;
let shape = "", nodeEls = {}, links = [], linkEls = [];
let hist = {}, reqHist = [], particles = [], raf = 0, lastSpawnSec = null;

// ---------------------------------------------------------------------------
// The model: which boxes, in which column, joined how. Rebuilt from scratch on
// every tick; the DOM is only rebuilt when its shape changes.
// ---------------------------------------------------------------------------
function stageOf(s) {
  if (s.state === "locked") return "locked";
  if (s.link !== "streaming") return s.link || "no audio";
  if (!s.tone_detected) return "no tick";
  if (!s.phase_locked) return "no edge";
  if (!s.anchored) return "no frame";
  if (s.refusal && s.refusal !== "none") return s.refusal;
  return "voting";
}

// A source's part in the served time: in it, measured and waiting, or neither.
const srcState = (s) => s.in_use ? "live" : (s.ready || s.have_offset) ? "standby" : "down";

function spark(name) {
  const h = hist[name];
  if (!h || h.length < 2) return '<svg class="fl-spark" viewBox="0 0 100 22" preserveAspectRatio="none"></svg>';
  // Symmetric about zero, never tighter than ±2 ms, so a flat line near zero
  // looks flat rather than like noise blown up to fill the box.
  const m = Math.max(2, ...h.map(Math.abs));
  const pts = h.map((v, i) => (i * 100 / (h.length - 1)).toFixed(1) + "," + (11 - v / m * 9).toFixed(1));
  return '<svg class="fl-spark" viewBox="0 0 100 22" preserveAspectRatio="none">' +
    '<line x1="0" x2="100" y1="11" y2="11"/>' +
    '<polyline points="' + pts.join(" ") + '"/></svg>';
}

function metrics(rows) {
  return '<div class="fl-m">' + rows.filter(Boolean).map((r) =>
    '<div><i>' + r[0] + '</i><b' + (r[2] ? ' class="' + r[2] + '"' : "") + ">" + r[1] + "</b></div>").join("") +
    "</div>";
}

const toneOf = (a, warn, bad) => !fin(a) ? "" : Math.abs(a) >= bad ? "fl-bad" : Math.abs(a) >= warn ? "fl-warn" : "";

function model(d) {
  const cols = [[], [], [], []], L = [];
  const served = d.offset_ms;
  const radio = d.sources.filter((s) => s.kind !== "ntp");
  const peers = d.sources.filter((s) => s.kind === "ntp");
  // Failing back is the primary class coming back, so its countdown goes on
  // the primary's own time signals -- the ready ones it will come back on. The
  // figure is written by paintCountdowns() in index.html, which interpolates
  // it smoothly between ticks; see setCountdown there.
  const k = d.clock || {};
  const backPill = k.failback_in_seconds != null ? ["", "warn fl-cdback"] : null;

  // Column 0, radio: one box per transmitter, however many receivers hear it.
  const stations = {};
  for (const s of radio) {
    const st = status[s.name] || {};
    let id = s.station && STATIONS[s.station] ? s.station : "unknown";
    if (id === "unknown" && st.carrier_hz === 77500) id = "dcf77";
    else if (id === "unknown" && st.carrier_hz === 162000) id = "allouis";
    else if (id === "unknown" && st.carrier_hz && st.carrier_hz < 1e6) id = "wwvb";
    (stations[id] = stations[id] || []).push(s);
  }
  for (const [id, list] of Object.entries(stations)) {
    const heard = list.filter((s) => s.tone_detected);
    const freqs = [...new Set(list.map((s) => (status[s.name] || {}).carrier_hz).filter(Boolean))]
      .sort((a, b) => a - b).map(freq);
    cols[0].push({
      key: "st:" + id, kind: "rf", state: heard.length ? "live" : "down", serving: list.some((s) => s.in_use),
      title: STATIONS[id].name, tag: freqs.join(" · ") || "HF",
      sub: list.every((s) => s.link === "stopped") ? "no receiver running" : STATIONS[id].where,
      // Only a station with a ready source is one the class comes back on:
      // heard is not enough, a carrier can be heard with no minute decoding.
      pill: k.primary === "radio" && list.some((s) => s.ready) ? backPill : null,
      kf: heard.length + "/" + list.length, kl: "heard",
      body: metrics([
        ["heard by", heard.length + " of " + list.length],
        ["best tick", heard.length ? num(Math.max(...heard.map((s) => s.tone_snr_db))) + " dB" : "—"],
      ]),
    });
  }

  // Column 0, network: each peer's own reference, which is what its time is
  // worth; this daemon cannot see further up than that.
  for (const s of peers) {
    const n = s.ntp || {};
    const ok = n.stratum > 0 && n.stratum < 16;
    cols[0].push({
      key: "ref:" + s.name, kind: "ntp", state: ok ? "live" : "down", serving: s.in_use,
      title: n.refid || "?", tag: ok ? "stratum " + (n.stratum - 1) : "unknown",
      sub: !ok ? "no reply yet" : n.stratum === 1 ? "reference clock" : "upstream server",
      pill: k.primary === "ntp" && ok ? backPill : null,
      kf: msv(n.root_distance_ms, 2), kl: "root dist",
      body: metrics([["root dist", msv(n.root_distance_ms, 2)]]),
    });
    L.push({ from: "ref:" + s.name, to: "src:" + s.name, kind: "ntp", state: ok ? "live" : "down", serving: s.in_use,
             label: ok ? "stratum " + n.stratum : "" });
  }

  // Column 1: the receivers and peers themselves.
  for (const s of radio) {
    const st = status[s.name] || {}, dl = st.delay || {};
    const state = srcState(s);
    const stage = stageOf(s);
    const station = STATIONS[s.station] ? STATIONS[s.station].name : "?";
    const vs = s.have_offset ? s.offset_ms - served : null;
    const km = /([\d,.]+) km/.exec(dl.path || "");
    // The propagation model says which it assumed: groundwave (LF, or HF close
    // in) or skywave hops. The link is labelled by that, not always "sky".
    const ground = /groundwave/.test(dl.path || "");
    // DCF77 times the second from PM when it can and AM when it cannot.
    const dcf = (st.decoder || {}).dcf77;
    cols[1].push({
      key: "src:" + s.name, kind: "ws", state, serving: s.in_use, title: s.name,
      tag: station + (st.carrier_hz ? " " + freq(st.carrier_hz) : ""),
      sub: st.receiver_name || st.url || "UberSDR receiver",
      hint: (st.url || "") + (s.not_used_reason ? "\nnot used: " + s.not_used_reason : "") +
        (dl.path ? "\n" + dl.path : ""),
      pill: [stage, s.state === "locked" ? "ok" : s.link !== "streaming" ? "bad" : "warn"],
      kf: s.have_offset ? sgn(vs) + " ms" : "—", kfc: toneOf(vs, 10, 50), kl: stage,
      body: metrics([
        ["vs served", s.have_offset ? sgn(vs) + " ms" : "—", toneOf(vs, 10, 50)],
        ["tick SNR", s.tone_detected ? num(s.tone_snr_db) + " dB" : "—"],
        dcf && ["timing from", dcf.timing === "pm" ? "PM" : "AM"],
        dcf && ["PM", !dcf.pm_locked ? "searching"
          : fin(dcf.pm_snr_db) ? num(dcf.pm_snr_db) + " dB" : "tracking"],
        ["decode", s.last_quality ? s.last_quality + "%" : "—"],
        ["frames voted", s.window_size ? s.frames_in_window + " / " + s.window_size : "—"],
        [ground ? "ground path" : "sky path", km ? km[1] + " km" : "—"],
        ["network", fin(dl.network_ms) && s.link === "streaming" ? num(dl.network_ms) + " ms" : "—"],
      ]) + spark(s.name),
    });
    const id = s.station && STATIONS[s.station] ? s.station
      : st.carrier_hz === 77500 ? "dcf77"
      : st.carrier_hz === 162000 ? "allouis"
      : (st.carrier_hz && st.carrier_hz < 1e6 ? "wwvb" : "unknown");
    L.push({ from: "st:" + id, to: "src:" + s.name, kind: "rf",
             state: s.tone_detected ? "live" : "down", serving: s.in_use,
             label: fin(dl.propagation_ms) && dl.propagation_ms > 0
               ? (ground ? "ground " : "sky ") +
                 // Two decimals when short, so a 0.98 ms groundwave hop does not read 1.0.
                 num(dl.propagation_ms, dl.propagation_ms < 10 ? 2 : 1) + " ms" : "" });
    L.push({ from: "src:" + s.name, to: "core", kind: "ws", serving: s.in_use,
             state: s.link !== "streaming" ? "down" : state,
             label: s.link === "streaming" && fin(dl.network_ms) ? "net " + num(dl.network_ms) + " ms" : "",
             hint: "WebSocket audio; one-way network delay, half the smallest recent round trip" });
  }
  for (const s of peers) {
    const n = s.ntp || {};
    const state = s.ntp && s.ntp.kiss_code ? "down" : srcState(s);
    const vs = s.have_offset ? s.offset_ms - served : null;
    let bits = "";
    for (let i = 7; i >= 0; --i) bits += '<span class="' + (((n.reach || 0) >> i) & 1 ? "on" : "") + '"></span>';
    cols[1].push({
      key: "src:" + s.name, kind: "ntp", state, serving: s.in_use, title: s.name,
      tag: "NTP" + (n.stratum ? " · stratum " + n.stratum : ""),
      sub: n.address || n.server || "",
      hint: s.not_used_reason ? "not used: " + s.not_used_reason : "",
      pill: n.kiss_code ? [n.kiss_code, "bad"] : [s.in_use ? "in use" : s.state, s.state === "locked" ? "ok" : "warn"],
      kf: s.have_offset ? sgn(vs) + " ms" : "—", kfc: toneOf(vs, 10, 50), kl: "reach " + (n.reach_octal || "000"),
      body: metrics([
        ["vs served", s.have_offset ? sgn(vs) + " ms" : "—", toneOf(vs, 10, 50)],
        ["round trip", msv(n.delay_ms, 2)],
        ["reach", '<span class="fl-reach">' + bits + "</span>"],
        ["poll", n.poll_seconds ? n.poll_seconds.toFixed(0) + " s" : "—"],
      ]) + spark(s.name),
    });
    L.push({ from: "src:" + s.name, to: "core", kind: "ntp", state, serving: s.in_use,
             label: fin(n.delay_ms) && n.delay_ms > 0 ? "UDP " + num(n.delay_ms, 2) + " ms" : "" });
  }

  // Column 2: this daemon.
  const cd = k.class_delta || {};
  // Ticks are sent on the second boundary and can land a hair either side.
  const sec = Math.floor(d.unix + 0.05) % 60;
  cols[2].push({
    key: "core", kind: "core", state: d.synchronised ? "live" : "down",
    // Not the product name: the column heading already says what this stage
    // is, the page title already says what is running, and a reader following
    // signals across the diagram wants the ROLE of each box. Every other node
    // here names a role or a station rather than a program.
    title: "This server",
    // The reference as the page states it everywhere else: every station in
    // the answer, not just the one the refid had room to name. index.html owns
    // that helper; guarded so this file still draws if it is ever loaded alone.
    sub: d.synchronised ? "stratum " + d.stratum + " · " + refOf(d) + " · v" + (d.version || "") : "unsynchronised",
    pill: k.serving === "secondary" ? ["failed over to " + k.secondary, "bad"]
      : k.serving === "both" ? ["serving both classes", "warn"]
      : k.failover_in_seconds != null ? ["failing over in " + Math.ceil(k.failover_in_seconds) + " s", "warn"]
      : k.failback_in_seconds != null ? ["failing back in " + Math.ceil(k.failback_in_seconds) + " s", "warn"]
      : d.synchronised ? ["serving " + (k.primary || "time"), "ok"] : ["not serving", "bad"],
    ring: sec,
    kf: "±" + msv(d.dispersion_ms, 0), kfc: toneOf(d.dispersion_ms, 50, 250),
    kl: d.synchronised ? "stratum " + d.stratum + " · " + refOf(d) : "unsynchronised",
    body: '<div class="fl-from"><i>time from</i>' + ((d.used_names || []).length
      ? d.used_names.map((n) => '<span>' + esc(n) + "</span>").join("") : "<em>nothing in use</em>") + "</div>" +
      metrics([
      ["host offset", sgn(d.offset_ms) + " ms"],
      ["dispersion", "±" + msv(d.dispersion_ms), toneOf(d.dispersion_ms, 50, 250)],
      ["sources used", d.sources_used + " of " + d.sources_candidate],
      cd.valid ? [esc(k.primary) + " − " + esc(k.secondary), sgn(cd.delta_ms) + " ms", toneOf(cd.delta_ms, 10, 50)]
        : ["classes", "not compared"],
      ["host drift", fin(d.clock_rate_ppm) ? sgn(d.clock_rate_ppm, 2) + " ppm" : "—"],
      ["newest fix", fin(d.reference_age_seconds) ? num(d.reference_age_seconds, 0) + " s ago" : "—"],
    ]),
  });

  // Column 3: what it serves.
  const ntp = d.ntp || {};
  const rate = reqRate();
  cols[3].push({
    key: "ntpc", kind: "out", state: !d.synchronised ? "down" : ntp.requests ? "live" : "standby",
    title: "NTP server", tag: "UDP :" + ntp.port,
    sub: d.synchronised ? "ours, answering at stratum " + d.stratum : "answering unsynchronised",
    kf: (ntp.requests || 0).toLocaleString(), kl: "requests",
    body: metrics([
      ["requests", (ntp.requests || 0).toLocaleString()],
      ["per minute", rate == null ? "—" : num(rate, rate < 10 ? 1 : 0)],
      ["answered", (ntp.answered || 0).toLocaleString()],
      ["refused", ((ntp.ignored || 0) + (ntp.rate_limited || 0)).toLocaleString()],
    ]),
  });
  L.push({ from: "core", to: "ntpc", kind: "out", serving: d.synchronised, state: !d.synchronised ? "down" : ntp.requests ? "live" : "standby",
           label: rate == null ? "" : num(rate, rate < 10 ? 1 : 0) + "/min" });

  const b = browser;
  const dev = !b ? "—" : Math.abs(b.device) <= b.within ? "±" + num(b.within, b.within < 1 ? 1 : 0) + " ms"
    : (Math.abs(b.device) < 1000 ? num(Math.abs(b.device), 0) + " ms" : num(Math.abs(b.device) / 1000) + " s") +
      (b.device > 0 ? " fast" : " slow");
  cols[3].push({
    key: "web", kind: "out", state: !linkOk ? "down" : b ? "live" : "standby",
    title: "This browser", short: "Browser", tag: "HTTP", sub: "the event stream and /api/time",
    kf: b ? msv(b.rtt, 1) : "—", kl: b ? "round trip" : linkOk ? "measuring" : "offline",
    body: metrics([
      ["round trip", b ? msv(b.rtt, 1) : "—"],
      ["likely", b && fin(b.likely) ? "±" + msv(b.likely, 1) : "—"],
      ["at most", b ? "±" + msv(b.err, 1) : "—"],
      ["your clock", dev, b && Math.abs(b.device) > Math.max(20, b.within) ? (Math.abs(b.device) < 500 ? "fl-warn" : "fl-bad") : ""],
    ]),
  });
  L.push({ from: "core", to: "web", kind: "out", serving: linkOk, state: !linkOk ? "down" : b ? "live" : "standby",
           label: b ? "RTT " + num(b.rtt) + " ms" : linkOk ? "measuring" : "offline" });

  return { cols, links: L };
}

function reqRate() {
  if (reqHist.length < 2) return null;
  const a = reqHist[0], z = reqHist[reqHist.length - 1];
  if (z.t - a.t < 5000) return null;
  return Math.max(0, z.n - a.n) / (z.t - a.t) * 60000;
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------
// See stationMix() in index.html: the refid can only name one station, but the
// served time is often a consensus over two. Guarded so this file degrades to
// the bare refid rather than throwing if it is ever drawn without the page.
function refOf(d) {
  try {
    if (typeof stationMix === "function") {
      const r = stationMix(d);
      if (r && r.text) return r.text;
    }
  } catch (x) {}
  return d.refid || "";
}

const HEADS = ["Time signals", "Receivers & servers", "Disciplined clock", "Outputs"];
const SHORT_HEADS = ["Signals", "Receivers", "Clock", "Outputs"];

function build(m) {
  grid.textContent = "";
  nodeEls = {};
  m.cols.forEach((col, i) => {
    const c = document.createElement("div");
    c.className = "fl-col";
    c.innerHTML = '<div class="fl-head"><span>0' + (i + 1) + '</span><em>' + HEADS[i] + "</em><u>" +
      SHORT_HEADS[i] + "</u></div>";
    for (const n of col) {
      const el = document.createElement("div");
      el.className = "fl-node fl-k-" + n.kind;
      // A mouse only: a touch "enters" and never leaves, and would leave the
      // rest of the drawing dimmed.
      el.addEventListener("pointerenter", (e) => { if (e.pointerType === "mouse") focus(n.key); });
      el.addEventListener("pointerleave", () => focus(null));
      el.addEventListener("click", () => { selected = n.key; showDetail(); });
      nodeEls[n.key] = el;
      c.appendChild(el);
    }
    grid.appendChild(c);
  });

  svg.textContent = "";
  chipLayer.textContent = "";
  linkEls = m.links.map((l) => {
    const g = document.createElementNS(NS, "g");
    const mk = (cls) => {
      const p = document.createElementNS(NS, "path");
      p.setAttribute("class", cls);
      g.appendChild(p);
      return p;
    };
    const glow = mk("fl-glow"), line = mk("fl-line"), flow = mk("fl-flow");
    svg.appendChild(g);
    const chip = document.createElement("span");
    chip.className = "fl-chip";
    chipLayer.appendChild(chip);
    return { g, glow, line, flow, chip, len: 0 };
  });
}

function paint(m) {
  for (const col of m.cols) for (const n of col) {
    const el = nodeEls[n.key];
    el.classList.remove("is-live", "is-standby", "is-down");
    el.classList.add("is-" + n.state);
    el.classList.toggle("is-serving", !!n.serving && n.kind !== "core" && n.kind !== "out");
    el.title = n.hint || "";
    el.classList.toggle("fl-sel", n.key === selected);
    kindOf[n.key] = n.kind;
    el.innerHTML = fullHtml[n.key] =
      (n.serving && (n.kind === "ws" || n.kind === "ntp") && n.key.startsWith("src:")
        ? '<span class="fl-badge">serving</span>' : "") +
      (n.ring != null
        ? '<div class="fl-ring"><svg viewBox="0 0 44 44"><circle class="t" cx="22" cy="22" r="19"/>' +
          '<circle class="p" cx="22" cy="22" r="19"/></svg><b>' + String(n.ring).padStart(2, "0") + "</b></div>"
        : "") +
      '<div class="fl-nh"><span class="fl-dot"></span><b class="fl-title">' +
        (n.short ? "<em>" + esc(n.title) + "</em><u>" + esc(n.short) + "</u>" : esc(n.title)) + "</b>" +
        (n.tag ? '<span class="fl-tag">' + esc(n.tag) + "</span>" : "") + "</div>" +
      // Only the compact form shows this: one figure that says the most.
      (n.kf != null ? '<div class="fl-kf"><b class="' + (n.kfc || "") + '">' + esc(n.kf) + "</b><i>" +
        esc(n.kl || "") + "</i></div>" : "") +
      (n.sub ? '<div class="fl-sub">' + esc(n.sub) + "</div>" : "") +
      (n.pill ? '<span class="fl-pill ' + n.pill[1] + '">' + esc(n.pill[0]) + "</span>" : "") +
      n.body;
  }
  m.links.forEach((l, i) => {
    const e = linkEls[i];
    const hue = l.state === "down" ? HUE.bad : HUE[l.kind];
    e.g.setAttribute("class", "fl-link is-" + l.state + (l.serving && l.state === "live" ? " is-serving" : ""));
    e.g.dataset.from = l.from;
    e.g.dataset.to = l.to;
    e.g.style.setProperty("--c", hue);
    e.chip.style.setProperty("--c", hue);
    e.chip.textContent = l.label;
    e.chip.hidden = !l.label;
    e.chip.dataset.from = l.from;
    e.chip.dataset.to = l.to;
  });
  links = m.links;
  showDetail();
}

// In the compact form the boxes carry one figure each; the full box for
// whichever one was tapped sits under the diagram.
function showDetail() {
  if (!detail) return;
  if (!fullHtml[selected]) selected = "core";
  for (const [k, el] of Object.entries(nodeEls)) el.classList.toggle("fl-sel", k === selected);
  const src = nodeEls[selected];
  detail.innerHTML = '<div class="' + (src ? src.className.replace(/\bfl-(sel|on)\b/g, "") : "fl-node") + '">' +
    (fullHtml[selected] || "") + "</div>";
}

// Where each link leaves and arrives. Several links on one side of a box are
// spread along it in the order of their other ends, so they fan out rather
// than cross or pile onto one point.
function layout() {
  if (!root || !links.length) return;
  // Below this the full boxes no longer fit four abreast. Measured on the
  // scrolling container, whose width does not depend on the choice.
  const compact = scroller.clientWidth < 940;
  if (compact !== root.parentNode.parentNode.classList.contains("fl-compact")) {
    root.parentNode.parentNode.classList.toggle("fl-compact", compact);
  }
  const base = root.getBoundingClientRect();
  const W = base.width, H = base.height;
  if (!W || !H) return;
  svg.setAttribute("viewBox", "0 0 " + W + " " + H);
  svg.setAttribute("width", W);
  svg.setAttribute("height", H);
  const dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(W * dpr) || canvas.height !== Math.round(H * dpr)) {
    canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
    canvas.style.width = W + "px"; canvas.style.height = H + "px";
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  const box = {};
  for (const [k, el] of Object.entries(nodeEls)) {
    const r = el.getBoundingClientRect();
    box[k] = { x: r.left - base.left, y: r.top - base.top, w: r.width, h: r.height };
  }
  const mid = (b) => b.y + b.h / 2;
  const sides = {};
  links.forEach((l, i) => {
    (sides[l.from + ">"] = sides[l.from + ">"] || []).push({ i, other: l.to });
    (sides[l.to + "<"] = sides[l.to + "<"] || []).push({ i, other: l.from });
  });
  const at = {};
  for (const [side, list] of Object.entries(sides)) {
    const key = side.slice(0, -1), out = side.endsWith(">"), b = box[key];
    if (!b) continue;
    list.sort((p, q) => mid(box[p.other] || b) - mid(box[q.other] || b));
    const n = list.length;
    const step = n > 1 ? Math.min(14, b.h * 0.6 / (n - 1)) : 0;
    list.forEach((p, j) => {
      at[p.i + (out ? "a" : "b")] =
        { x: out ? b.x + b.w : b.x, y: b.y + b.h / 2 + (j - (n - 1) / 2) * step };
    });
  }

  links.forEach((l, i) => {
    const a = at[i + "a"], z = at[i + "b"], e = linkEls[i];
    if (!a || !z) return;
    const dx = Math.max(18, (z.x - a.x) * 0.55);
    const c1 = { x: a.x + dx, y: a.y }, c2 = { x: z.x - dx, y: z.y };
    const d = "M" + a.x + " " + a.y + " C" + c1.x + " " + c1.y + " " + c2.x + " " + c2.y + " " + z.x + " " + z.y;
    for (const p of [e.glow, e.line, e.flow]) p.setAttribute("d", d);
    e.len = e.line.getTotalLength();
    // The bezier's own midpoint, for the label.
    e.chip.style.left = (a.x + 3 * c1.x + 3 * c2.x + z.x) / 8 + "px";
    e.chip.style.top = (a.y + 3 * c1.y + 3 * c2.y + z.y) / 8 + "px";
  });
}

function focus(key) {
  root.classList.toggle("fl-focus", !!key);
  const near = new Set(key ? [key] : []);
  if (key) for (const l of links) {
    if (l.from === key) near.add(l.to);
    if (l.to === key) near.add(l.from);
  }
  for (const [k, el] of Object.entries(nodeEls)) el.classList.toggle("fl-on", near.has(k));
  for (const e of linkEls) {
    const on = !!key && (e.g.dataset.from === key || e.g.dataset.to === key);
    e.g.classList.toggle("fl-on", on);
    e.chip.classList.toggle("fl-on", on);
  }
}

// ---------------------------------------------------------------------------
// The pulse: on each broadcast second, a spark leaves every transmitter and is
// handed down the chain column by column, so the second visibly travels from
// the station to the page. Standby links carry a fainter one -- they are
// measured, just not listened to. Down links carry nothing.
// ---------------------------------------------------------------------------
const COL_OF = (key) => key.startsWith("st:") || key.startsWith("ref:") ? 0 : key.startsWith("src:") ? 1 : 2;
// Each hop is handed on as the one before it lands.
const kHopMs = 440, kHopGapMs = 440;

function spawn() {
  if (reduced.matches || document.hidden) return;
  const now = performance.now();
  links.forEach((l, i) => {
    if (l.state === "down") return;
    const t0 = now + COL_OF(l.from) * kHopGapMs;
    particles.push({ e: linkEls[i], t0, dur: kHopMs,
                     hue: HUE[l.kind], weak: l.state !== "live", big: l.serving && l.state === "live" });
  });
  if (!raf) raf = requestAnimationFrame(frame);
}

const ease = (t) => t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;

function frame(now) {
  raf = 0;
  const W = canvas.width, H = canvas.height;
  ctx.save(); ctx.setTransform(1, 0, 0, 1, 0, 0); ctx.clearRect(0, 0, W, H); ctx.restore();
  particles = particles.filter((p) => now < p.t0 + p.dur + 200);
  ctx.globalCompositeOperation = "lighter";
  for (const p of particles) {
    if (now < p.t0 || !p.e.len) continue;
    const t = Math.min(1, (now - p.t0) / p.dur);
    const a = p.weak ? 0.4 : 1;
    // A short comet: the head and a tail of fading points behind it.
    for (let k = 7; k >= 0; --k) {
      const tt = t - k * 0.028;
      if (tt < 0) continue;
      const pt = p.e.line.getPointAtLength(p.e.len * ease(tt));
      const r = (p.weak ? 1.6 : p.big ? 3 : 2.3) * (1 - k / 9);
      ctx.globalAlpha = a * (1 - k / 8) * (t >= 1 ? Math.max(0, 1 - (now - p.t0 - p.dur) / 200) : 1);
      ctx.fillStyle = p.hue;
      ctx.beginPath(); ctx.arc(pt.x, pt.y, r, 0, Math.PI * 2); ctx.fill();
      if (k === 0) {
        const g = ctx.createRadialGradient(pt.x, pt.y, 0, pt.x, pt.y, r * 6);
        g.addColorStop(0, p.hue); g.addColorStop(1, "transparent");
        ctx.globalAlpha *= 0.45;
        ctx.fillStyle = g;
        ctx.beginPath(); ctx.arc(pt.x, pt.y, r * 6, 0, Math.PI * 2); ctx.fill();
      }
    }
  }
  ctx.globalAlpha = 1;
  ctx.globalCompositeOperation = "source-over";
  if (particles.length) raf = requestAnimationFrame(frame);
}

// ---------------------------------------------------------------------------
// Setup and the three entry points index.html calls.
// ---------------------------------------------------------------------------
function mount() {
  const card = document.getElementById("flowcard");
  if (!card) return false;
  card.classList.add("fl-card");
  card.innerHTML =
    '<div class="fl-top"><div class="fl-label">Signal path</div>' +
    '<div class="fl-sum" id="flsum"></div>' +
    '<div class="fl-legend"><span class="live">feeding the time</span>' +
    '<span class="standby">measured, standing by</span><span class="down">down</span></div></div>' +
    '<div class="fl-scroll"><div class="fl"><svg class="fl-links" aria-hidden="true"></svg><canvas class="fl-fx" aria-hidden="true"></canvas>' +
    '<div class="fl-grid"></div><div class="fl-chips" aria-hidden="true"></div></div></div>' +
    '<div class="fl-detail"></div>';
  root = card.querySelector(".fl");
  grid = card.querySelector(".fl-grid");
  svg = card.querySelector(".fl-links");
  canvas = card.querySelector(".fl-fx");
  ctx = canvas.getContext("2d");
  chipLayer = card.querySelector(".fl-chips");
  summary = card.querySelector("#flsum");
  detail = card.querySelector(".fl-detail");
  scroller = card.querySelector(".fl-scroll");
  const ro = new ResizeObserver(layout);
  ro.observe(root);
  ro.observe(scroller);
  return true;
}

function render() {
  if (!tick || (!root && !mount())) return;
  const m = model(tick);
  const s = m.cols.map((c) => c.map((n) => n.key).join(",")).join("|") + "#" +
    m.links.map((l) => l.from + ">" + l.to).join(",");
  if (s !== shape) { shape = s; build(m); }
  paint(m);
  layout();
  // Fill the failback pills now rather than on the page's next 250 ms paint,
  // so a freshly drawn box never flashes an empty one.
  if (typeof paintCountdowns === "function") paintCountdowns();

  const radio = tick.sources.filter((x) => x.kind !== "ntp");
  const peers = tick.sources.filter((x) => x.kind === "ntp");
  const k = tick.clock || {};
  const used = tick.used_names || [];
  const via = [...new Set(tick.sources.filter((x) => x.in_use).map((x) =>
    x.kind === "ntp" ? "NTP" : STATIONS[x.station] ? STATIONS[x.station].name : "radio"))];
  summary.innerHTML = (tick.synchronised && used.length
    ? '<b class="fl-now">Serving ' + esc(via.join(" + ")) + "</b> via " + esc(used.join(", ")) + " · "
    : '<b class="fl-now bad">Not serving</b> · ') + esc(
    radio.filter((x) => x.in_use).length + " of " + radio.length + " receiver" + (radio.length === 1 ? "" : "s") +
    (peers.length ? " · " + peers.filter((x) => x.in_use).length + " of " + peers.length + " NTP peer" +
      (peers.length === 1 ? "" : "s") : "") + " in use" +
    (k.serving === "secondary" ? " · failed over to " + k.secondary : ""));
}

window.Flow = {
  tick(d) {
    tick = d;
    const now = performance.now();
    for (const s of d.sources) {
      const h = hist[s.name] = hist[s.name] || [];
      if (s.have_offset && fin(s.offset_ms) && fin(d.offset_ms)) h.push(s.offset_ms - d.offset_ms);
      if (h.length > 120) h.shift();
    }
    if (d.ntp) {
      reqHist.push({ t: now, n: d.ntp.requests || 0 });
      while (reqHist.length > 2 && now - reqHist[0].t > 60000) reqHist.shift();
    }
    render();
    // One pulse per broadcast second, however the ticks bunch.
    const sec = Math.floor(d.unix);
    if (sec !== lastSpawnSec) { lastSpawnSec = sec; spawn(); }
  },
  status(d) {
    status = {};
    for (const s of d.sources || []) status[s.name] = s;
  },
  browser(b) { browser = b; },
  link(ok) {
    linkOk = ok;
    if (!ok) { browser = null; render(); }
  },
};

// ---------------------------------------------------------------------------
// Styles. The panel is dark in both themes on purpose: it is the one piece of
// the page meant to be looked at rather than read, and the glow only works
// against a dark ground.
// ---------------------------------------------------------------------------
const css = `
.fl-card { position:relative; overflow:hidden; color:#e4e9f7; border-color:#1d2744;
  background:
    radial-gradient(900px 340px at 0% -20%, rgba(56,189,248,.16), transparent 60%),
    radial-gradient(700px 360px at 100% 120%, rgba(167,139,250,.18), transparent 60%),
    linear-gradient(rgba(148,163,255,.045) 1px, transparent 1px) 0 0/100% 26px,
    linear-gradient(90deg, rgba(148,163,255,.045) 1px, transparent 1px) 0 0/26px 100%,
    #0a0f1f; }
.fl-top { display:flex; flex-wrap:wrap; align-items:baseline; gap:6px 16px; margin-bottom:14px; }
.fl-label { font-size:11px; text-transform:uppercase; letter-spacing:.1em; font-weight:700;
  background:linear-gradient(90deg,#38bdf8,#a78bfa); -webkit-background-clip:text; background-clip:text; color:transparent; }
.fl-sum { font-size:12px; color:#8d99bf; margin-right:auto; }
.fl-legend { display:flex; flex-wrap:wrap; gap:4px 14px; font-size:11px; color:#8d99bf; }
.fl-legend span::before { content:""; display:inline-block; width:18px; height:0; vertical-align:middle;
  margin-right:6px; border-top:2px solid #22d3ee; }
.fl-legend .standby::before { border-top-style:dashed; opacity:.6; }
.fl-legend .down::before { border-top:2px dotted #fb7185; opacity:.7; }

.fl-scroll { overflow-x:auto; margin:0 -16px; padding:10px 16px 4px; }
.fl { position:relative; min-width:780px; }
.fl-compact .fl { min-width:0; }
.fl-links, .fl-fx, .fl-chips { position:absolute; inset:0; pointer-events:none; }
.fl-links { overflow:visible; z-index:0; }
.fl-fx { z-index:1; }
.fl-chips { z-index:3; }
.fl-grid { position:relative; z-index:2; display:grid; align-items:center;
  grid-template-columns:minmax(0,.85fr) minmax(0,1.2fr) minmax(0,1.1fr) minmax(0,1fr);
  column-gap:clamp(38px,6.5vw,92px); }
.fl-col { display:flex; flex-direction:column; gap:18px; min-width:0; }
.fl-head { font-size:10px; text-transform:uppercase; letter-spacing:.12em; font-weight:600; color:#6f7ca6;
  margin-bottom:2px; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
.fl-head span { color:#38bdf8; margin-right:7px; font-family:var(--mono); }
.fl-head em, .fl-head u { font-style:normal; text-decoration:none; }
.fl-head u { display:none; }
.fl-title em, .fl-title u { font-style:normal; text-decoration:none; } .fl-title u { display:none; }
.fl-compact .fl-head em, .fl-compact .fl-grid .fl-title em { display:none; }
.fl-compact .fl-head u, .fl-compact .fl-grid .fl-title u { display:inline; }

.fl-node { --c:#22d3ee; position:relative; min-width:0; border-radius:12px; padding:10px 12px 11px;
  background:linear-gradient(180deg,rgba(24,33,60,.92),rgba(14,20,40,.92));
  border:1px solid rgba(148,163,255,.14);
  box-shadow:0 12px 30px -16px rgba(0,0,0,.8), inset 0 1px 0 rgba(255,255,255,.04);
  transition:border-color .3s, box-shadow .3s, opacity .25s, transform .25s; }
.fl-k-rf { --c:#38bdf8; } .fl-k-ws { --c:#22d3ee; } .fl-k-ntp { --c:#a78bfa; } .fl-k-out { --c:#34d399; }
.fl-node.is-live { border-color:color-mix(in srgb,var(--c) 42%,transparent);
  box-shadow:0 0 26px -10px var(--c), 0 12px 30px -16px rgba(0,0,0,.8), inset 0 1px 0 rgba(255,255,255,.05); }
.fl-node.is-standby { border-style:dashed; border-color:color-mix(in srgb,var(--c) 30%,transparent); }
.fl-node.is-down { --c:#fb7185; opacity:.62; }
.fl-focus .fl-node { opacity:.35; }
.fl-focus .fl-node.fl-on { opacity:1; transform:translateY(-1px); }

.fl-nh { display:flex; align-items:center; gap:7px; min-width:0; }
.fl-dot { flex:none; width:8px; height:8px; border-radius:50%; background:var(--c); box-shadow:0 0 8px var(--c); }
.is-live .fl-dot { animation:fl-beat 1s ease-out; }
.is-standby .fl-dot { background:transparent; border:1.5px solid var(--c); box-shadow:none; }
@keyframes fl-beat { 0% { box-shadow:0 0 0 0 color-mix(in srgb,var(--c) 70%,transparent), 0 0 8px var(--c); }
  100% { box-shadow:0 0 0 7px transparent, 0 0 8px var(--c); } }
.fl-kf { display:none; }
.fl-title { font:600 13px/1.2 var(--mono); white-space:nowrap; overflow:hidden; text-overflow:ellipsis; min-width:0; }
.fl-tag { margin-left:auto; flex:none; font:600 9.5px/1 var(--mono); padding:3px 6px; border-radius:6px;
  color:var(--c); background:color-mix(in srgb,var(--c) 14%,transparent); white-space:nowrap; }
.fl-sub { font-size:11px; color:#8391b9; margin-top:4px; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
.fl-pill { display:inline-block; margin-top:7px; font-size:9.5px; font-weight:700; text-transform:uppercase;
  letter-spacing:.07em; padding:2px 8px; border-radius:99px; }
.fl-pill.ok { color:#4ade80; background:rgba(74,222,128,.13); }
.fl-pill.warn { color:#fbbf24; background:rgba(251,191,36,.14); }
.fl-pill.bad { color:#fb7185; background:rgba(251,113,133,.15); }
.fl-m { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:6px 10px; margin-top:8px; }
.fl-m div { min-width:0; }
.fl-m i { display:block; font-style:normal; font-size:9px; text-transform:uppercase; letter-spacing:.08em;
  color:#aab5d8; white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
.fl-m b { display:block; font:600 12.5px/1.3 var(--mono); font-variant-numeric:tabular-nums; color:#eef2ff;
  white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
.fl-m b.fl-warn { color:#fbbf24; } .fl-m b.fl-bad { color:#fb7185; }
.fl-reach { display:inline-flex; gap:2px; vertical-align:middle; }
.fl-reach span { width:5px; height:9px; border-radius:1.5px; background:rgba(148,163,255,.18); }
.fl-reach span.on { background:var(--c); box-shadow:0 0 5px var(--c); }
.fl-spark { display:block; width:100%; height:22px; margin-top:8px; overflow:visible; }
.fl-spark line { stroke:rgba(148,163,255,.18); stroke-width:1; vector-effect:non-scaling-stroke; }
.fl-spark polyline { fill:none; stroke:var(--c); stroke-width:1.4; vector-effect:non-scaling-stroke;
  filter:drop-shadow(0 0 3px var(--c)); stroke-linejoin:round; }

/* the daemon: the one box everything goes through */
.fl-k-core { --c:#22d3ee; padding:14px 14px 13px; border:1px solid transparent; isolation:isolate;
  background:linear-gradient(180deg,#141d38,#0d1429) padding-box,
             linear-gradient(135deg,#38bdf8,#a78bfa 55%,#34d399) border-box; }
@property --fl-a { syntax:"<angle>"; inherits:false; initial-value:0deg; }
@keyframes fl-spin { to { --fl-a:360deg; } }
.fl-k-core .fl-title { font-size:15px; background:linear-gradient(90deg,#7dd3fc,#c4b5fd);
  -webkit-background-clip:text; background-clip:text; color:transparent; }
.fl-k-core .fl-tag { margin-left:0; }
.fl-k-core .fl-nh { padding-right:52px; }
.fl-k-core .fl-sub { padding-right:52px; }
.fl-ring { position:absolute; top:10px; right:10px; width:44px; height:44px; }
.fl-ring svg { width:100%; height:100%; transform:rotate(-90deg); }
.fl-ring circle { fill:none; stroke-width:3; }
.fl-ring .t { stroke:rgba(148,163,255,.15); }
.fl-ring .p { stroke:#38bdf8; stroke-linecap:round; stroke-dasharray:119.4;
  stroke-dashoffset:119.4; filter:drop-shadow(0 0 4px #38bdf8); animation:fl-sweep 1s linear forwards; }
.fl-k-core.is-down .fl-ring .p { stroke:#fb7185; animation:none; }
@keyframes fl-sweep { to { stroke-dashoffset:0; } }
.fl-ring b { position:absolute; inset:0; display:grid; place-items:center; font:600 13px/1 var(--mono);
  font-variant-numeric:tabular-nums; color:#e0f2fe; }

.fl-link path { fill:none; stroke:var(--c); transition:opacity .25s; }
.fl-glow { stroke-width:7; opacity:.10; }
.fl-line { stroke-width:1.5; opacity:.55; }
.fl-flow { stroke-width:2; stroke-dasharray:2 10; stroke-linecap:round; opacity:.9;
  animation:fl-dash .9s linear infinite; }
.fl-link.is-standby .fl-line { stroke-dasharray:5 5; opacity:.4; }
.fl-link.is-standby .fl-glow { opacity:.04; }
.fl-link.is-standby .fl-flow { opacity:.35; animation-duration:2.2s; }
.fl-link.is-down .fl-line { stroke-dasharray:1.5 5; opacity:.45; }
.fl-link.is-down .fl-glow, .fl-link.is-down .fl-flow { display:none; }
@keyframes fl-dash { to { stroke-dashoffset:-12; } }
.fl-focus .fl-link path { opacity:.06; }
.fl-focus .fl-link.fl-on .fl-line { opacity:.9; } .fl-focus .fl-link.fl-on .fl-glow { opacity:.22; }
.fl-focus .fl-link.fl-on .fl-flow { opacity:1; }

.fl-chip { position:absolute; transform:translate(-50%,-50%); font:600 10px/1 var(--mono);
  font-variant-numeric:tabular-nums; color:var(--c); white-space:nowrap; padding:3px 7px; border-radius:99px;
  background:rgba(9,14,30,.9); border:1px solid color-mix(in srgb,var(--c) 35%,transparent);
  box-shadow:0 0 12px -4px var(--c); transition:opacity .25s; }
.fl-focus .fl-chip { opacity:.15; } .fl-focus .fl-chip.fl-on { opacity:1; }


/* the source(s) the served time is actually coming from */
.fl-node.is-serving { border:1.5px solid transparent;
  background:linear-gradient(180deg,rgba(24,36,66,.97),rgba(14,22,44,.97)) padding-box,
             conic-gradient(from var(--fl-a,0deg),var(--c),rgba(255,255,255,.1) 25%,var(--c) 50%,rgba(255,255,255,.1) 75%,var(--c)) border-box;
  animation:fl-spin 6s linear infinite;
  box-shadow:0 0 34px -8px var(--c), 0 12px 30px -16px rgba(0,0,0,.8); }
.fl-badge { position:absolute; top:-9px; right:12px; z-index:1; font:700 9px/1 ui-sans-serif,system-ui,sans-serif;
  text-transform:uppercase; letter-spacing:.12em; padding:4px 8px 4px 16px; border-radius:99px; color:#04121a;
  background:linear-gradient(90deg,var(--c),#e0f2fe); box-shadow:0 0 14px -2px var(--c); }
.fl-badge::before { content:""; position:absolute; left:6px; top:50%; width:5px; height:5px; margin-top:-2.5px;
  border-radius:50%; background:#04121a; animation:fl-blink 1s steps(2,start) infinite; }
@keyframes fl-blink { to { visibility:hidden; } }
.fl-link.is-serving .fl-glow { stroke-width:11; opacity:.2; }
.fl-link.is-serving .fl-line { stroke-width:2.4; opacity:.85; }
.fl-link.is-serving .fl-flow { stroke-width:2.6; stroke-dasharray:3 8; animation-duration:.6s; }
.fl-from { display:flex; flex-wrap:wrap; align-items:center; gap:5px; margin-top:9px; }
.fl-from i { font-style:normal; font-size:9px; text-transform:uppercase; letter-spacing:.08em; color:#aab5d8; margin-right:2px; }
.fl-from span { font:600 11px/1 var(--mono); padding:4px 7px; border-radius:6px; color:#e0f2fe;
  background:linear-gradient(90deg,rgba(56,189,248,.28),rgba(167,139,250,.28)); border:1px solid rgba(125,211,252,.35); }
.fl-from em { font-style:normal; color:#fb7185; font-size:12px; }
.fl-now { color:#7dd3fc; font-weight:700; } .fl-now.bad { color:#fb7185; }



/* the compact form: the same left-to-right drawing, one figure per box */
.fl-compact .fl-grid { column-gap:clamp(14px,4.5vw,40px); }
.fl-compact .fl-col { gap:14px; }
.fl-compact .fl-head { font-size:8.5px; letter-spacing:.08em; }
.fl-compact .fl-head span { display:none; }
.fl-compact .fl-grid .fl-node { padding:7px 8px 8px; border-radius:10px; cursor:pointer; }
.fl-compact .fl-grid :is(.fl-sub,.fl-pill,.fl-m,.fl-spark,.fl-from,.fl-tag) { display:none; }
.fl-compact .fl-grid { grid-template-columns:minmax(0,.9fr) minmax(0,1fr) minmax(0,1.05fr) minmax(0,1.05fr); }
.fl-compact .fl-grid .fl-title { font-size:10.5px; white-space:normal; overflow-wrap:break-word; line-height:1.25; }
.fl-compact .fl-grid .fl-nh { gap:5px; }
.fl-compact .fl-grid .fl-dot { position:absolute; top:6px; right:6px; width:6px; height:6px; }
.fl-compact .fl-grid .fl-kf { display:block; margin-top:4px; min-width:0; }
.fl-compact .fl-kf b { display:block; font:600 11px/1.25 var(--mono); letter-spacing:-.03em; font-variant-numeric:tabular-nums; color:#eef2ff;
  white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
.fl-compact .fl-kf b.fl-warn { color:#fbbf24; } .fl-compact .fl-kf b.fl-bad { color:#fb7185; }
.fl-compact .fl-kf i { display:block; font-style:normal; font-size:9px; color:#aab5d8; white-space:nowrap;
  overflow:hidden; text-overflow:ellipsis; }
.fl-compact .fl-grid .fl-badge { top:-7px; right:6px; font-size:7px; padding:3px 5px 3px 11px; letter-spacing:.08em; }
.fl-compact .fl-grid .fl-badge::before { left:4px; width:4px; height:4px; margin-top:-2px; }
.fl-compact .fl-grid .fl-k-core { padding-top:8px; }
.fl-compact .fl-grid .fl-ring { position:relative; top:auto; right:auto; width:40px; height:40px; margin:0 auto 5px; }
.fl-compact .fl-grid .fl-k-core .fl-nh { justify-content:center; padding-right:0; }
.fl-compact .fl-grid .fl-k-core .fl-dot { top:9px; right:9px; }
.fl-compact .fl-grid .fl-k-core .fl-title { font-size:10.5px; }
.fl-compact .fl-grid .fl-k-core .fl-kf { text-align:center; }
.fl-compact .fl-grid .fl-node.fl-sel { outline:1.5px solid color-mix(in srgb,var(--c) 70%,transparent); outline-offset:2px; }
.fl-compact .fl-chip { display:none; }
.fl-compact .fl-scroll { overflow:visible; }
.fl-detail { display:none; }
.fl-compact .fl-detail { display:block; margin-top:18px; }
.fl-compact .fl-detail .fl-node { animation:none; }
.fl-compact .fl-detail::after { content:"tap a box above for its detail"; display:block; margin-top:8px;
  font-size:10px; color:#6f7ca6; text-align:right; }
@media (prefers-reduced-motion: reduce) {
  .fl-flow, .is-live .fl-dot, .fl-ring .p, .fl-node.is-serving, .fl-badge::before { animation:none; }
  .fl-ring .p { stroke-dashoffset:0; }
}
`;
const style = document.createElement("style");
style.textContent = css;
document.head.appendChild(style);
})();
