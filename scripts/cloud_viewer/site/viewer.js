/*
 * cloudPointRPC point-cloud viewer.
 *
 * Public API (called by generated files):
 *   CloudViewer.setManifest({title, generated, frames:[{id, label, has_gt, metrics}]})
 *   CloudViewer.addFrame(id, base64Payload)
 *
 * Frames are loaded lazily by inserting <script src="frames/<id>.js">; that file
 * calls CloudViewer.addFrame(). No fetch(), no modules, no network: works from file://.
 *
 * Payload "CPV1" (little-endian): "CPV1", u16 width, u16 height, u16 stride,
 * u8 flags (bit0 = GT, bit1 = RGB), u8 reserved, int16 xyz[w*h*3] (0.02 mm units,
 * -32768 = invalid), int16 gt_xyz[w*h*3] if bit0, u8 rgb[w*h*3] if bit1.
 */
(function () {
  'use strict';

  var Q = 0.02;          // mm per unit
  var INVALID = -32768;
  var ID_RE = /^[A-Za-z0-9_.-]+$/;
  var NEUTRAL = [0.33, 0.36, 0.39];  // points without a GT counterpart in error/bias modes

  var manifest = null;        // as passed to setManifest
  var frames = [];            // manifest.frames
  var byId = {};              // id -> manifest frame entry
  var cache = {};             // id -> parsed payload
  var pending = null;         // id requested but not yet added
  var requested = {};         // id -> true once a <script> was inserted
  var cur = null;             // currently displayed parsed frame
  var curId = null;
  var mode = 'error';
  var gl = null;              // three.js state (renderer, scene, camera, controls, materials)
  var recPts = null, gtPts = null;

  function $(id) { return document.getElementById(id); }
  function css(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }
  function esc(s) { return String(s).replace(/[&<>"]/g, function (c) { return {'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;'}[c]; }); }
  function has(v) { return v !== null && v !== undefined && typeof v === 'number' && isFinite(v); }
  function fmtMm(v, d) { return has(v) ? (v * 1000).toFixed(d) + ' mm' : '—'; }
  function fmtPct(v, d) { return has(v) ? (v * 100).toFixed(d) + ' %' : '—'; }
  function fmtMs(v) { return has(v) ? v.toFixed(0) + ' ms' : '—'; }
  function numFmt(n) { return n.toLocaleString(); }

  // ---------------------------------------------------------------- metrics
  // Summary table columns. A column is shown when at least one frame has the metric.
  var COLUMNS = [
    {key: 'valid_fraction', header: 'Valid px',     fmt: function (v) { return fmtPct(v, 1); }},
    {key: 'coverage',       header: 'GT coverage',  fmt: function (v) { return fmtPct(v, 1); }},
    {key: 'z_median_m',     header: 'Median depth', fmt: function (v) { return fmtMm(v, 0); }},
    {key: 'mae_3d_m',       header: 'MAE 3D',       fmt: function (v) { return fmtMm(v, 2); }, bar: true},
    {key: 'median_3d_m',    header: 'Median err',   fmt: function (v) { return fmtMm(v, 2); }},
    {key: 'rmse_3d_m',      header: 'RMSE 3D',      fmt: function (v) { return fmtMm(v, 2); }},
    {key: 'within_1mm',     header: '< 1 mm',       fmt: function (v) { return fmtPct(v, 1); }},
    {key: 'within_2mm',     header: '< 2 mm',       fmt: function (v) { return fmtPct(v, 1); }},
    {key: 'within_5mm',     header: '< 5 mm',       fmt: function (v) { return fmtPct(v, 1); }},
    {key: 'matching_ms',    header: 'Matching',     fmt: fmtMs}
  ];

  // Per-frame metric list. Composite rows are shown when any part is present.
  var KV_ROWS = [
    {label: 'Valid pixels', parts: [['', 'valid_fraction']], fmt: function (v) { return fmtPct(v, 1); }},
    {label: 'Depth', parts: [['p5', 'z_p05_m'], ['p50', 'z_median_m'], ['p95', 'z_p95_m']], fmt: function (v) { return (v * 1000).toFixed(0); }, unit: ' mm'},
    {label: 'Matching', parts: [['', 'matching_ms']], fmt: fmtMs},
    {label: 'Reconstruction', parts: [['', 'reconstruction_ms']], fmt: fmtMs},
    {label: 'GT coverage', parts: [['', 'coverage']], fmt: function (v) { return fmtPct(v, 1); }},
    {label: 'MAE', parts: [['3D', 'mae_3d_m'], ['z', 'mae_z_m']], fmt: function (v) { return (v * 1000).toFixed(2); }, unit: ' mm'},
    {label: 'RMSE 3D', parts: [['', 'rmse_3d_m']], fmt: function (v) { return fmtMm(v, 2); }},
    {label: 'Median 3D error', parts: [['', 'median_3d_m']], fmt: function (v) { return fmtMm(v, 2); }},
    {label: 'Within', parts: [['1', 'within_1mm'], ['2', 'within_2mm'], ['5', 'within_5mm']], fmt: function (v) { return (v * 100).toFixed(0); }, unit: ' %', suffix: ' mm'}
  ];

  function metricsOf(f) { return (f && f.metrics) || {}; }
  function frameLabel(f) { return f.label || f.id; }
  function frameHasGt(f) { return f.has_gt === true || has(metricsOf(f).mae_3d_m); }

  function gradePill(m) {
    if (!has(m.mae_3d_m)) return '<span class="pill na">no GT</span>';
    var mae = m.mae_3d_m * 1000;
    return mae < 1 ? '<span class="pill good">good</span>' : mae <= 1.5 ? '<span class="pill warn">fair</span>' : '<span class="pill bad">weak</span>';
  }

  function buildTable() {
    var cols = COLUMNS.filter(function (c) { return frames.some(function (f) { return has(metricsOf(f)[c.key]); }); });
    var anyGrade = frames.some(function (f) { return has(metricsOf(f).mae_3d_m); });
    var maxMae = 0;
    frames.forEach(function (f) { var v = metricsOf(f).mae_3d_m; if (has(v) && v > maxMae) maxMae = v; });

    var th = '<th>Frame</th>' + cols.map(function (c) { return '<th>' + esc(c.header) + '</th>'; }).join('') + (anyGrade ? '<th>Grade</th>' : '');
    $('thead-row').innerHTML = th;
    var tbody = $('tbody');
    tbody.innerHTML = '';
    frames.forEach(function (f) {
      var m = metricsOf(f);
      var tr = document.createElement('tr');
      tr.dataset.id = f.id;
      var html = '<td>' + esc(frameLabel(f)) + '</td>';
      cols.forEach(function (c) {
        var v = m[c.key];
        var cell = c.fmt(v);
        if (c.bar && has(v) && maxMae > 0) cell += '<span class="bar" style="width:' + (v / maxMae * 48).toFixed(0) + 'px"></span>';
        html += '<td' + (has(v) ? '' : ' class="dim"') + '>' + cell + '</td>';
      });
      if (anyGrade) html += '<td>' + gradePill(m) + '</td>';
      tr.innerHTML = html;
      tr.addEventListener('click', function () {
        selectFrame(f.id);
        var reduce = window.matchMedia && matchMedia('(prefers-reduced-motion: reduce)').matches;
        $('vtitle').scrollIntoView({behavior: reduce ? 'auto' : 'smooth', block: 'start'});
      });
      tbody.appendChild(tr);
    });
    $('grade-note').hidden = !anyGrade;
    $('frames-heading').textContent = frames.length === 1 ? 'Frame' : 'All frames (' + frames.length + ')';
  }

  function buildSelect() {
    var sel = $('frame');
    sel.innerHTML = '';
    frames.forEach(function (f) {
      var o = document.createElement('option');
      o.value = f.id; o.textContent = frameLabel(f);
      sel.appendChild(o);
    });
    $('frame-row').hidden = frames.length < 2;
  }

  function kv(f) {
    var m = metricsOf(f), out = '';
    KV_ROWS.forEach(function (r) {
      var present = r.parts.filter(function (p) { return has(m[p[1]]); });
      if (!present.length) return;
      var names = present.map(function (p) { return p[0]; }).filter(Boolean);
      var label = r.label + (names.length ? ' ' + names.join(' / ') + (r.suffix || '') : '');
      var val = present.map(function (p) { return r.fmt(m[p[1]]); }).join(' / ') + (r.unit || '');
      out += '<dt>' + esc(label) + '</dt><dd>' + esc(val) + '</dd>';
    });
    $('kv').innerHTML = out;
  }

  // ---------------------------------------------------------------- payload
  function decodeBase64(b64) {
    var bin = atob(b64.replace(/\s+/g, ''));
    var u8 = new Uint8Array(bin.length);
    for (var i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
    return u8.buffer;
  }

  function parseCPV1(buf) {
    if (buf.byteLength < 12) throw new Error('payload too short');
    var dv = new DataView(buf);
    var magic = String.fromCharCode(dv.getUint8(0), dv.getUint8(1), dv.getUint8(2), dv.getUint8(3));
    if (magic !== 'CPV1') throw new Error('bad magic "' + magic + '" (expected CPV1)');
    var W = dv.getUint16(4, true), H = dv.getUint16(6, true), stride = dv.getUint16(8, true), flags = dv.getUint8(10);
    var n = W * H, hasGt = !!(flags & 1), hasRgb = !!(flags & 2);
    var need = 12 + 6 * n + (hasGt ? 6 * n : 0) + (hasRgb ? 3 * n : 0);
    if (buf.byteLength < need) throw new Error('payload truncated: ' + buf.byteLength + ' < ' + need + ' bytes');
    var off = 12;
    var rec = new Int16Array(buf, off, 3 * n); off += 6 * n;
    var gt = null, rgb = null;
    if (hasGt) { gt = new Int16Array(buf, off, 3 * n); off += 6 * n; }
    if (hasRgb) { rgb = new Uint8Array(buf, off, 3 * n); }
    return {W: W, H: H, stride: stride, n: n, rec: rec, gt: gt, rgb: rgb};
  }

  // Build positions / per-point error arrays once per frame.
  function prepare(d) {
    var recIdx = [], gtIdx = [], i, j;
    for (i = 0; i < d.n; i++) {
      if (d.rec[3 * i] !== INVALID) recIdx.push(i);
      if (d.gt && d.gt[3 * i] !== INVALID) gtIdx.push(i);
    }
    function toPos(arr, idx) {
      var p = new Float32Array(idx.length * 3);
      for (var k = 0; k < idx.length; k++) {
        var s = idx[k];
        p[3 * k] = arr[3 * s] * Q; p[3 * k + 1] = -arr[3 * s + 1] * Q; p[3 * k + 2] = -arr[3 * s + 2] * Q;  // y down, z forward -> three.js y up, z towards viewer
      }
      return p;
    }
    d.recIdx = recIdx; d.gtIdx = gtIdx;
    d.recPos = toPos(d.rec, recIdx);
    d.gtPos = d.gt ? toPos(d.gt, gtIdx) : null;
    d.err = new Float32Array(recIdx.length); d.dz = new Float32Array(recIdx.length);
    var zs = new Float32Array(recIdx.length), xs = new Float32Array(recIdx.length), ys = new Float32Array(recIdx.length);
    d.scored = 0;
    for (j = 0; j < recIdx.length; j++) {
      i = recIdx[j];
      xs[j] = d.rec[3 * i] * Q; ys[j] = -d.rec[3 * i + 1] * Q; zs[j] = d.rec[3 * i + 2] * Q;
      if (d.gt && d.gt[3 * i] !== INVALID) {
        var dx = (d.rec[3 * i] - d.gt[3 * i]) * Q, dy = (d.rec[3 * i + 1] - d.gt[3 * i + 1]) * Q, dzz = (d.rec[3 * i + 2] - d.gt[3 * i + 2]) * Q;
        d.err[j] = Math.sqrt(dx * dx + dy * dy + dzz * dzz); d.dz[j] = dzz; d.scored++;
      } else { d.err[j] = NaN; d.dz[j] = NaN; }
    }
    var zsort = Float32Array.from(zs).sort(), xsort = Float32Array.from(xs).sort(), ysort = Float32Array.from(ys).sort();
    var L = zsort.length;
    if (L) {
      d.z5 = zsort[Math.floor(L * 0.05)]; d.z95 = zsort[Math.min(L - 1, Math.floor(L * 0.95))]; d.zmed = zsort[L >> 1];
      d.center = new THREE.Vector3(xsort[L >> 1], ysort[L >> 1], -d.zmed);
    } else { d.z5 = 0; d.z95 = 1; d.zmed = 0; d.center = new THREE.Vector3(0, 0, 0); }
    if (d.z95 <= d.z5) d.z95 = d.z5 + 1;
    d.ready = true;
  }

  // ---------------------------------------------------------------- colour ramps
  var VIRIDIS = [[68,1,84],[72,40,120],[62,74,137],[49,104,142],[38,130,142],[31,158,137],[53,183,121],[109,205,89],[180,222,44],[253,231,37]];
  var TURBO = [[48,18,59],[70,107,227],[40,187,236],[49,242,153],[162,252,60],[237,208,58],[251,128,34],[210,48,5],[122,4,3]];
  var RDBU = [[33,102,172],[103,169,207],[209,229,240],[247,247,247],[253,219,199],[239,138,98],[178,24,43]];
  function ramp(stops, t, out, o) {
    t = t < 0 ? 0 : t > 1 ? 1 : (t === t ? t : 0);
    var x = t * (stops.length - 1), i = Math.min(stops.length - 2, Math.floor(x)), f = x - i, a = stops[i], b = stops[i + 1];
    out[o] = (a[0] + (b[0] - a[0]) * f) / 255; out[o + 1] = (a[1] + (b[1] - a[1]) * f) / 255; out[o + 2] = (a[2] + (b[2] - a[2]) * f) / 255;
  }
  function rampCss(s) { return 'linear-gradient(90deg,' + s.map(function (c, i) { return 'rgb(' + c.join(',') + ') ' + (i / (s.length - 1) * 100).toFixed(0) + '%'; }).join(',') + ')'; }
  function rampRgb(stops, t) { var o = [0, 0, 0]; ramp(stops, t, o, 0); return 'rgb(' + o.map(function (v) { return Math.round(v * 255); }).join(',') + ')'; }

  function clampLimit() { return +$('clamp').value; }

  function colours(d) {
    var c = new Float32Array(d.recIdx.length * 3), lim = clampLimit(), span = d.z95 - d.z5;
    for (var j = 0; j < d.recIdx.length; j++) {
      var i = d.recIdx[j], o = 3 * j;
      if (mode === 'rgb' && d.rgb) { c[o] = d.rgb[3 * i] / 255; c[o + 1] = d.rgb[3 * i + 1] / 255; c[o + 2] = d.rgb[3 * i + 2] / 255; }
      else if (mode === 'depth' || mode === 'rgb') ramp(TURBO, (d.rec[3 * i + 2] * Q - d.z5) / span, c, o);
      else if (d.err[j] !== d.err[j]) { c[o] = NEUTRAL[0]; c[o + 1] = NEUTRAL[1]; c[o + 2] = NEUTRAL[2]; }
      else if (mode === 'error') ramp(VIRIDIS, d.err[j] / lim, c, o);
      else ramp(RDBU, 0.5 + d.dz[j] / (2 * lim), c, o);
    }
    return c;
  }

  function legend(d) {
    var lg = $('legend'), lim = clampLimit();
    var spec = {
      error: {stops: VIRIDIS, lo: '0', mid: '3D error, mm', hi: '≥ ' + lim},
      dz: {stops: RDBU, lo: '−' + lim, mid: 'z bias, mm', hi: '+' + lim},
      depth: {stops: TURBO, lo: d.z5.toFixed(0), mid: 'depth, mm', hi: d.z95.toFixed(0)}
    }[mode];
    lg.hidden = !spec;
    if (!spec) return;
    lg.innerHTML = '<div class="ramp" style="background:' + rampCss(spec.stops) + '"></div>' +
      '<div class="ticks"><span>' + spec.lo + '</span><span>' + spec.mid + '</span><span>' + spec.hi + '</span></div>' +
      (mode !== 'depth' && d.gt ? '<div style="margin-top:3px">grey = no ground truth</div>' : '');
  }

  // ---------------------------------------------------------------- three.js
  function initGL() {
    if (gl) return true;
    var stage = $('stage');
    if (typeof THREE === 'undefined') { showLoading('vendor/three.min.js did not load', true); return false; }
    var renderer;
    try { renderer = new THREE.WebGLRenderer({antialias: true}); }
    catch (e) { showLoading('WebGL is not available in this browser: ' + e.message, true); return false; }
    renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    renderer.setClearColor(new THREE.Color(css('--canvas') || '#11161a'), 1);
    stage.insertBefore(renderer.domElement, stage.firstChild);
    var scene = new THREE.Scene();
    var camera = new THREE.PerspectiveCamera(40, 1.25, 0.5, 5000);
    var controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    var recMat = new THREE.PointsMaterial({size: +$('psize').value, vertexColors: true, sizeAttenuation: false});
    var gtMat = new THREE.PointsMaterial({size: +$('psize').value, color: new THREE.Color(css('--gt') || '#c2410c'), sizeAttenuation: false, transparent: true, opacity: 0.8});
    gl = {renderer: renderer, scene: scene, camera: camera, controls: controls, recMat: recMat, gtMat: gtMat};
    function resize() {
      var w = stage.clientWidth, h = stage.clientHeight;
      if (!w || !h) return;
      renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix();
    }
    resize();
    if (window.ResizeObserver) new ResizeObserver(resize).observe(stage); else window.addEventListener('resize', resize);
    (function tick() { controls.update(); renderer.render(scene, camera); requestAnimationFrame(tick); })();
    if (window.matchMedia) {
      matchMedia('(prefers-color-scheme: dark)').addEventListener('change', function () {
        renderer.setClearColor(new THREE.Color(css('--canvas')), 1); gtMat.color.set(css('--gt'));
      });
    }
    return true;
  }

  function frameView(view) {
    if (!cur || !gl) return;
    var c = cur.center, span = (cur.z95 - cur.z5) + 60, cam = gl.camera;
    gl.controls.target.copy(c);
    if (view === 'side') cam.position.set(c.x + Math.max(span * 1.6, cur.zmed * 1.5), c.y + 10, c.z + 10);
    else cam.position.set(c.x + cur.zmed * 0.5, c.y + cur.zmed * 0.4, c.z + cur.zmed * 1.5);  // oblique, from the camera side
    cam.up.set(0, 1, 0); gl.controls.update();
    gl.renderer.render(gl.scene, gl.camera);  // draw once now; the rAF loop keeps it up to date
  }

  function recolour() {
    if (!cur || !recPts) return;
    recPts.geometry.setAttribute('color', new THREE.BufferAttribute(colours(cur), 3));
    legend(cur);
  }

  function applyVis() {
    if (recPts) recPts.visible = $('showrec').checked;
    if (gtPts) gtPts.visible = $('showgt').checked && !$('showgt').disabled;
  }

  function setMode(m, re) {
    mode = m;
    var btns = document.querySelectorAll('.seg button');
    for (var i = 0; i < btns.length; i++) btns[i].setAttribute('aria-pressed', btns[i].dataset.mode === m ? 'true' : 'false');
    if (re !== false) recolour();
  }

  function setAvailability(d) {
    var hasGt = !!d.gt, hasRgb = !!d.rgb;
    $('m-error').disabled = !hasGt; $('m-dz').disabled = !hasGt; $('m-rgb').disabled = !hasRgb;
    $('m-error').title = hasGt ? '' : 'No ground truth for this frame';
    $('m-dz').title = $('m-error').title;
    $('m-rgb').title = hasRgb ? '' : 'No image colour in this frame';
    $('showgt').disabled = !hasGt;
    $('showgt-label').classList.toggle('off', !hasGt);
    $('showgt-label').title = hasGt ? '' : 'No ground truth for this frame';
    if (!hasGt && (mode === 'error' || mode === 'dz')) setMode('depth', false);
    if (!hasRgb && mode === 'rgb') setMode('depth', false);
  }

  // ---------------------------------------------------------------- display
  function showLoading(text, isError) {
    var el = $('loading');
    el.hidden = false; el.textContent = text; el.classList.toggle('err', !!isError);
  }

  function display(id) {
    var d = cache[id], f = byId[id] || {id: id};
    if (!initGL()) return;
    if (!d.ready) prepare(d);
    if (recPts) { gl.scene.remove(recPts); recPts.geometry.dispose(); recPts = null; }
    if (gtPts) { gl.scene.remove(gtPts); gtPts.geometry.dispose(); gtPts = null; }
    cur = d; curId = id;
    var g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(d.recPos, 3));
    recPts = new THREE.Points(g, gl.recMat); gl.scene.add(recPts);
    if (d.gtPos) { var gg = new THREE.BufferGeometry(); gg.setAttribute('position', new THREE.BufferAttribute(d.gtPos, 3)); gtPts = new THREE.Points(gg, gl.gtMat); gl.scene.add(gtPts); }
    setAvailability(d);
    applyVis(); recolour(); frameView();
    $('vtitle').textContent = 'Cloud viewer · ' + frameLabel(f);
    $('hud').innerHTML = '<span>' + numFmt(d.recIdx.length) + ' recon pts' + (d.gt ? ' · ' + numFmt(d.gtIdx.length) + ' GT pts' : '') + '</span>' +
      '<span>median depth ' + d.zmed.toFixed(1) + ' mm · ' + d.W + '×' + d.H + ' samples, stride ' + d.stride + '</span>';
    $('foot-frame').textContent = 'Coordinates are in the rectified left-camera frame (x right, y down, z forward), in millimetres, quantised to 0.02 mm. ' +
      'This frame shows every ' + ordinal(d.stride) + ' pixel in each direction (' + d.W + ' × ' + d.H + ' = ' + numFmt(d.n) + ' samples, ' + numFmt(d.recIdx.length) + ' valid).';
    kv(f); charts(d);
    $('loading').hidden = true;
    var rows = $('tbody').children;
    for (var i = 0; i < rows.length; i++) rows[i].classList.toggle('sel', rows[i].dataset.id === id);
    if ($('frame').value !== id) $('frame').value = id;
    try { if (frames.length > 1 && history.replaceState) history.replaceState(null, '', '#' + id); } catch (e) { /* file:// may refuse */ }
  }

  function ordinal(n) { var s = ['th', 'st', 'nd', 'rd'], v = n % 100; return n + (s[(v - 20) % 10] || s[v] || s[0]); }

  function selectFrame(id) {
    if (!ID_RE.test(id)) return;
    pending = id;
    if (cache[id]) { display(id); return; }
    showLoading('Loading ' + frameLabel(byId[id] || {id: id}) + '…');
    if (requested[id]) return;  // script tag already inserted, addFrame will follow
    requested[id] = true;
    var s = document.createElement('script');
    s.src = 'frames/' + id + '.js';
    s.async = true;
    s.onerror = function () {
      requested[id] = false;
      if (pending === id) showLoading('Could not load frames/' + id + '.js', true);
    };
    document.body.appendChild(s);
  }

  // ---------------------------------------------------------------- charts (SVG)
  var X0 = 40, X1 = 390, Y0 = 140, Y1 = 12;
  function axes(xticks, xlab, yticks, ylab, sx, sy) {
    var s = '';
    yticks.forEach(function (t) { s += '<line x1="' + X0 + '" x2="' + X1 + '" y1="' + sy(t) + '" y2="' + sy(t) + '" stroke="var(--line)" stroke-width="1"/><text x="' + (X0 - 5) + '" y="' + (sy(t) + 3.5) + '" text-anchor="end">' + ylab(t) + '</text>'; });
    xticks.forEach(function (t) { s += '<text x="' + sx(t) + '" y="' + (Y0 + 14) + '" text-anchor="middle">' + t + '</text>'; });
    s += '<text class="sans" x="' + X1 + '" y="' + (Y0 + 28) + '" text-anchor="end">' + xlab + '</text>';
    return s;
  }
  function fmtTick(v) { return (Math.abs(v) >= 10 || v === Math.round(v)) ? String(Math.round(v)) : v.toFixed(1); }

  function charts(d) {
    var hist = $('hist'), bias = $('bias');
    if (!d.gt || !d.scored) {
      var msg = '<text class="msg" x="200" y="85" text-anchor="middle">No ground truth for this frame</text>';
      hist.innerHTML = msg; bias.innerHTML = msg; $('histnote').textContent = ''; $('biasnote').hidden = true;
      return;
    }
    $('biasnote').hidden = false;
    var lim = clampLimit();
    // histogram 0..2*lim, 32 bins
    var B = 32, range = 2 * lim, Wb = range / B, counts = new Array(B), n = 0, over = 0, j, e, worst = 0;
    for (j = 0; j < B; j++) counts[j] = 0;
    var sorted = new Float32Array(d.scored), k = 0;
    for (j = 0; j < d.err.length; j++) {
      e = d.err[j]; if (e !== e) continue;
      sorted[k++] = e; n++;
      if (e > worst) worst = e;
      if (e >= range) over++; else counts[Math.floor(e / Wb)]++;
    }
    sorted.sort();
    var med = sorted[n >> 1];
    var ymax = Math.max.apply(null, counts) / n, yTop = Math.max(0.05, Math.ceil(ymax * 100 / 5) * 5 / 100);
    var sx = function (v) { return X0 + (v / range) * (X1 - X0); }, sy = function (v) { return Y0 - (v / yTop) * (Y0 - Y1); };
    var s = axes([0, lim / 2, lim, lim * 1.5, lim * 2].map(fmtTick), '3D error, mm', [0, yTop / 2, yTop], function (v) { return (v * 100).toFixed(0) + '%'; },
      function (v) { return sx(+v); }, sy);
    var bw = (X1 - X0) / B - 1;
    counts.forEach(function (c, i) {
      s += '<rect x="' + (sx(i * Wb) + 0.5) + '" y="' + sy(c / n) + '" width="' + bw + '" height="' + (Y0 - sy(c / n)) + '" fill="' + rampRgb(VIRIDIS, (i + 0.5) * Wb / lim) + '"/>';
    });
    s += '<line x1="' + sx(med) + '" x2="' + sx(med) + '" y1="' + Y1 + '" y2="' + Y0 + '" stroke="var(--ink)" stroke-dasharray="3 3"/>' +
      '<text class="ink" x="' + (sx(med) + 4) + '" y="' + (Y1 + 10) + '">median ' + med.toFixed(2) + '</text>' +
      '<line x1="' + X0 + '" x2="' + X1 + '" y1="' + Y0 + '" y2="' + Y0 + '" stroke="var(--muted)"/>';
    hist.innerHTML = s;
    var mae = 0; for (j = 0; j < n; j++) mae += sorted[j]; mae /= n;
    $('histnote').textContent = numFmt(n) + ' scored samples, sample MAE ' + mae.toFixed(2) + ' mm. ' + (over / n * 100).toFixed(1) + ' % lie beyond ' + fmtTick(range) + ' mm (not drawn); worst ' + worst.toFixed(0) + ' mm.';

    // median depth bias per 10 mm GT-depth bin
    var bins = {};
    for (j = 0; j < d.recIdx.length; j++) {
      if (d.dz[j] !== d.dz[j]) continue;
      var z = d.gt[3 * d.recIdx[j] + 2] * Q, kb = Math.floor(z / 10) * 10;
      (bins[kb] || (bins[kb] = [])).push(d.dz[j]);
    }
    var pts = Object.keys(bins).filter(function (b) { return bins[b].length >= 150; }).map(function (b) {
      var v = bins[b]; v.sort(function (a, c) { return a - c; }); return [+b + 5, v[v.length >> 1], v.length];
    }).sort(function (a, b) { return a[0] - b[0]; });
    if (!pts.length) { bias.innerHTML = '<text class="msg" x="200" y="85" text-anchor="middle">Too few scored samples per depth bin</text>'; return; }
    var zlo = Math.min.apply(null, pts.map(function (p) { return p[0] - 5; })), zhi = Math.max.apply(null, pts.map(function (p) { return p[0] + 5; }));
    if (zhi - zlo < 40) { var mid = (zlo + zhi) / 2; zlo = Math.max(0, Math.floor((mid - 20) / 10) * 10); zhi = zlo + 40; }
    var maxAbs = Math.max.apply(null, pts.map(function (p) { return Math.abs(p[1]); }));
    var blim = Math.max(1, Math.min(10, Math.ceil(maxAbs * 1.25 * 2) / 2));
    var xstep = (zhi - zlo) <= 60 ? 10 : (zhi - zlo) <= 160 ? 20 : 40, xt = [];
    for (var t = Math.ceil(zlo / xstep) * xstep; t <= zhi; t += xstep) xt.push(t);
    var bx = function (v) { return X0 + (v - zlo) / (zhi - zlo) * (X1 - X0); }, by = function (v) { return Y0 - (v + blim) / (2 * blim) * (Y0 - Y1); };
    var u = axes(xt, 'ground-truth depth, mm', [-blim, -blim / 2, 0, blim / 2, blim], function (v) { return (v > 0 ? '+' : '') + fmtTick(v); }, bx, by);
    u += '<line x1="' + X0 + '" x2="' + X1 + '" y1="' + by(0) + '" y2="' + by(0) + '" stroke="var(--muted)"/>';
    var maxN = Math.max.apply(null, pts.map(function (p) { return p[2]; }));
    var cl = function (v) { return Math.max(-blim, Math.min(blim, v)); };
    u += '<polyline fill="none" stroke="var(--accent)" stroke-width="1.5" points="' + pts.map(function (p) { return bx(p[0]) + ',' + by(cl(p[1])); }).join(' ') + '"/>';
    pts.forEach(function (p) {
      u += '<circle cx="' + bx(p[0]) + '" cy="' + by(cl(p[1])) + '" r="' + (2.5 + 4 * Math.sqrt(p[2] / maxN)) + '" fill="var(--accent)" fill-opacity="0.85"><title>' +
        (p[0] - 5) + '–' + (p[0] + 5) + ' mm: ' + (p[1] > 0 ? '+' : '') + p[1].toFixed(2) + ' mm (' + p[2] + ' samples)</title></circle>';
    });
    bias.innerHTML = u;
  }

  // ---------------------------------------------------------------- wiring
  function wire() {
    var btns = document.querySelectorAll('.seg button');
    for (var i = 0; i < btns.length; i++) (function (b) { b.addEventListener('click', function () { if (!b.disabled) setMode(b.dataset.mode); }); })(btns[i]);
    $('showgt').addEventListener('change', applyVis);
    $('showrec').addEventListener('change', applyVis);
    $('psize').addEventListener('input', function (e) { if (gl) { gl.recMat.size = gl.gtMat.size = +e.target.value; } });
    $('clamp').addEventListener('input', function (e) { $('clampv').textContent = (+e.target.value).toFixed(1); recolour(); if (cur) charts(cur); });
    $('reset').addEventListener('click', function () { frameView(); });
    $('side').addEventListener('click', function () { frameView('side'); });
    $('frame').addEventListener('change', function (e) { selectFrame(e.target.value); });
    window.addEventListener('hashchange', function () { var id = hashId(); if (id && byId[id] && id !== curId) selectFrame(id); });
    setMode('error', false);
  }

  function hashId() {
    var h = (location.hash || '').replace(/^#/, '');
    try { h = decodeURIComponent(h); } catch (e) { /* keep raw */ }
    return h;
  }

  function disableControls() {
    var els = document.querySelectorAll('.controls button, .controls select, .controls input');
    for (var i = 0; i < els.length; i++) els[i].disabled = true;
    var ch = document.querySelector('.charts'); if (ch) ch.hidden = true;
  }

  function notice(html) { var n = $('notice'); n.innerHTML = html; n.hidden = false; }

  function setManifest(m) {
    manifest = m || {};
    frames = Array.isArray(manifest.frames) ? manifest.frames.filter(function (f) { return f && ID_RE.test(String(f.id)); }) : [];
    byId = {};
    frames.forEach(function (f) { byId[f.id] = f; });
    if (manifest.title) { document.title = manifest.title; $('title').textContent = manifest.title; }
    var meta = ['cloudPointRPC viewer'];
    if (manifest.generated) meta.push('generated ' + manifest.generated);
    meta.push(frames.length + (frames.length === 1 ? ' frame' : ' frames'));
    var withGt = frames.filter(frameHasGt).length;
    if (frames.length) meta.push(withGt + ' with ground truth');
    $('meta').textContent = meta.join(' · ');
    buildTable(); buildSelect();
    if (!frames.length) {
      notice('The manifest lists no frames. Nothing to show.');
      $('frames-section').hidden = true;
      disableControls();
      showLoading('No frames', true);
      return;
    }
    var start = hashId();
    if (!byId[start]) start = frames[0].id;
    selectFrame(start);
  }

  function addFrame(id, b64) {
    try {
      cache[id] = parseCPV1(decodeBase64(b64));
    } catch (e) {
      requested[id] = false;
      if (pending === id) showLoading('frames/' + id + '.js: ' + e.message, true);
      return;
    }
    if (pending === id) display(id);
  }

  wire();
  window.addEventListener('load', function () {
    if (manifest) return;
    notice('<code>frames/manifest.js</code> not found — build the site with <code>scripts/scared_overview.py --view</code> or CLI option 6.');
    $('frames-section').hidden = true;
    disableControls();
    showLoading('frames/manifest.js not found', true);
  });

  window.CloudViewer = {setManifest: setManifest, addFrame: addFrame};
})();
