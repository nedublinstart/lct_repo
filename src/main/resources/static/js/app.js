const state = {
  datasetId: null,
  jobId: null,
  poll: null,
  variants: [],
  ready: false,
  busy: false,
  uploadGen: 0,
};

let map = null;
const layers = {};

bindUi();
initMap();
setRunEnabled(false, "Сначала загрузите GeoJSON");

function stubLayer() {
  return {
    clearLayers() { return this; },
    addData() { return this; },
    addTo() { return this; },
    getLayers() { return []; },
  };
}

function bindUi() {
  document.getElementById("btn-demo").onclick = runDemo;
  document.getElementById("btn-contest").onclick = runContest;
  document.getElementById("btn-run").onclick = runJob;
  bindGuide();
  bindLegend();
  const file = document.getElementById("file");
  file.addEventListener("change", onFilePicked);
  const zone = document.getElementById("dropzone");
  ["dragenter", "dragover"].forEach((ev) => {
    zone.addEventListener(ev, (e) => {
      e.preventDefault();
      e.stopPropagation();
      zone.classList.add("drag");
    });
  });
  zone.addEventListener("dragleave", () => zone.classList.remove("drag"));
  zone.addEventListener("drop", (e) => {
    e.preventDefault();
    e.stopPropagation();
    zone.classList.remove("drag");
    const dropped = e.dataTransfer && e.dataTransfer.files && e.dataTransfer.files[0];
    if (!dropped) return;
    setDropzone("busy", dropped.name, "Отправляю файл на сервер…");
    uploadFile(dropped);
  });
  window.addEventListener("dragover", (e) => e.preventDefault());
  window.addEventListener("drop", (e) => {
    if (zone.contains(e.target)) return;
    e.preventDefault();
  });
}

function initMap() {
  document.querySelectorAll("input[data-layer]").forEach((el) => {
    el.addEventListener("change", () => {
      const layer = layers[el.dataset.layer];
      if (!map || !layer) return;
      if (el.checked) map.addLayer(layer);
      else map.removeLayer(layer);
    });
  });
  if (typeof L === "undefined") {
    layers.input = stubLayer();
    layers.new = stubLayer();
    layers.tap = stubLayer();
    layers.recon = stubLayer();
    setStatus("job-status", "Карта не загрузилась, расчёт всё равно можно запустить после загрузки файла.", "err");
    return;
  }
  map = L.map("map", { zoomControl: true }).setView([55.742, 37.585], 16);
  if (map.attributionControl) map.attributionControl.setPosition("bottomleft");
  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 20,
    attribution: "&copy; OpenStreetMap",
  }).addTo(map);
  layers.input = L.geoJSON(null, { style: styleInput, pointToLayer, onEachFeature }).addTo(map);
  layers.new = L.geoJSON(null, { style: styleResult, pointToLayer, onEachFeature }).addTo(map);
  layers.tap = L.layerGroup().addTo(map);
  layers.recon = L.layerGroup().addTo(map);
}

function kindOf(feature) {
  const p = feature.properties || {};
  return p.object_type || p.feature_type || p.restriction_type || "";
}

function styleInput(feature) {
  const t = kindOf(feature);
  const rt = (feature.properties || {}).restriction_type || "";
  if (t === "heat_network" || t === "existing_segment") return { color: "#5c6370", weight: 4 };
  if (t === "oks_future" || t === "oks_prospective") return { color: "#2a9d8f", weight: 2, fillOpacity: 0.25 };
  if (rt === "railway") return { color: "#4a4a4a", weight: 1, fillColor: "#666", fillOpacity: 0.35 };
  if (rt === "water") return { color: "#1d4e89", weight: 1, fillColor: "#7eb6d6", fillOpacity: 0.35 };
  if (rt === "road" || rt === "tdtp" || rt === "tram_tracks" || rt === "carriageway"
      || t === "road" || t === "tdtp") {
    return { color: "#6d7278", weight: 1, fillColor: "#b8bcc2", fillOpacity: 0.38 };
  }
  if (t === "restriction" || t === "constraint" || t === "oks" || t === "oks_existing" || rt === "oks") {
    return { color: "#8b6914", weight: 1, fillColor: "#c4a574", fillOpacity: 0.35 };
  }
  return { color: "#6b5848", weight: 1, fillOpacity: 0.2 };
}

function styleResult(feature) {
  const t = kindOf(feature);
  const p = feature.properties || {};
  const method = p.laying_method || "";
  const dn = Number(p.diameter) || 0;
  // Толщина линии — по DN, как в приложении: тонкая ветка и толстый ствол
  // не должны выглядеть одной ниткой. Спецпроход (дорога/ТДТП) — пунктир.
  const weight = dn >= 250 ? 8 : dn >= 200 ? 6.5 : dn >= 150 ? 5.5 : dn >= 125 ? 4.5 : 3.5;
  if ((t === "heat_network" || t === "new_segment") && method === "special") {
    return { color: "#9c3412", weight: Math.max(weight, 5), dashArray: "7 5", lineCap: "butt", lineJoin: "round" };
  }
  if (t === "heat_network" || t === "new_segment") {
    return { color: "#d04a1a", weight, lineCap: "round", lineJoin: "round" };
  }
  if (t === "heat_network_reconstruction" || t === "reconstruction_segment") {
    return { color: "#c9a227", weight: Math.max(4, weight - 1), dashArray: "8 6" };
  }
  return { color: "#1d3557", weight: 2 };
}

function pointToLayer(feature, latlng) {
  const t = kindOf(feature);
  // tie_in — единственная «врезка» на карте: крупный кружок с белой заливкой.
  // technical_node — стык участков, не врезка: мелкая точка, чтобы не плодить
  // фантомные маркеры вдоль каждой трубы.
  if (t === "tie_in" || t === "tap_point") {
    return L.circleMarker(latlng, {
      radius: 8,
      color: "#7a1f0d",
      fillColor: "#fff4ec",
      fillOpacity: 1,
      weight: 3,
    });
  }
  if (t === "technical_node") {
    return L.circleMarker(latlng, {
      radius: 3,
      color: "#8a7568",
      fillColor: "#8a7568",
      fillOpacity: 0.55,
      weight: 1,
    });
  }
  const colors = {
    heat_chamber: "#1d3557",
    chamber: "#1d3557",
    source: "#7a1f0d",
    oks_connection_point: "#2a9d8f",
    connection_point: "#2a9d8f",
    new_chamber: "#9b2226",
    heat_chamber_reconstruction: "#c9a227",
    reconstruction_chamber: "#c9a227",
  };
  return L.circleMarker(latlng, {
    radius: t === "source" ? 9 : 6,
    color: colors[t] || "#241910",
    fillColor: colors[t] || "#241910",
    fillOpacity: 0.9,
    weight: 2,
  });
}

const FIELD_LABELS = {
  id: "Идентификатор",
  name: "Название",
  variant_id: "Вариант",
  start_node_id: "Начало",
  end_node_id: "Конец",
  flow_tph: "Расход, т/ч",
  diameter: "Диаметр, мм",
  length: "Длина, м",
  laying_method: "Прокладка",
  cost: "Стоимость",
  depth_start: "Глубина начала, м",
  depth_end: "Глубина конца, м",
  restriction_type: "Ограничение",
  feature_type: "Тип объекта",
  special_reason: "Основание спецпрохода",
  reason: "Зачем узел",
  existing_object_id: "Существующий объект",
  existing_object_type: "Тип существующего",
  existing_diameter: "Было DN, мм",
  required_diameter: "Нужно DN, мм",
  existing_flow_tph: "Было, т/ч",
  added_flow_tph: "Добавлено, т/ч",
  calculated_flow_tph: "Стало, т/ч",
};

const TYPE_TITLES = {
  heat_network: "Новая труба",
  new_segment: "Новая труба",
  existing_segment: "Существующая сеть",
  heat_network_reconstruction: "Реконструкция участка",
  reconstruction_segment: "Реконструкция участка",
  tie_in: "Врезка",
  tap_point: "Врезка",
  technical_node: "Технический узел",
  heat_chamber: "Камера",
  chamber: "Камера",
  new_chamber: "Новая камера",
  heat_chamber_reconstruction: "Реконструкция камеры",
  reconstruction_chamber: "Реконструкция камеры",
  source: "Источник",
  oks_connection_point: "ИТП",
  connection_point: "ИТП",
  restriction: "Ограничение",
  oks: "Здание",
  oks_existing: "Здание",
  oks_future: "Перспективный ОКС",
  oks_prospective: "Перспективный ОКС",
  road: "Дорога",
  tdtp: "ТДТП",
};

const VALUE_WORDS = {
  base: "Обычная",
  special: "Спецпроход",
  road: "проезжая",
  tdtp: "ТДТП",
  tram_tracks: "трамвайные пути",
  carriageway: "проезжая",
  railway: "железная дорога",
  water: "вода",
  oks: "здание",
  heat_network: "теплосеть",
  heat_chamber: "камера",
  source: "источник",
  leave_special: "стык обычной прокладки и спецпрохода",
  itp_snap: "выход ИТП на фасад",
  steiner_branch: "ветвление трассы",
  island_stitch: "стык участков",
  diameter_step: "смена диаметра",
};

function onEachFeature(feature, layer) {
  const p = feature.properties || {};
  const t = kindOf(feature);
  const title = TYPE_TITLES[t] || (p.name ? String(p.name) : "Объект");
  const skip = { object_type: true };
  const rows = Object.keys(p).filter((k) => {
    if (skip[k]) return false;
    const v = p[k];
    return v !== null && v !== undefined && v !== "";
  }).map((k) => {
    const label = FIELD_LABELS[k] || k;
    return `<div><b>${escapeHtml(label)}</b> ${escapeHtml(formatValue(k, p[k]))}</div>`;
  }).join("");
  layer.bindPopup(`<div class="popup"><h3>${escapeHtml(title)}</h3>${rows}</div>`, { maxWidth: 320 });
}

function formatValue(key, value) {
  if (typeof value === "boolean") return value ? "да" : "нет";
  if (value && typeof value === "object") return JSON.stringify(value);
  const raw = String(value);
  if (key === "laying_method" || key === "restriction_type" || key === "special_reason"
      || key === "reason" || key === "existing_object_type" || key === "feature_type") {
    return VALUE_WORDS[raw] || raw;
  }
  const n = Number(value);
  if (!Number.isFinite(n) || raw.trim() === "") return raw;
  if (key === "cost") return money(n);
  if (key === "length" || key === "flow_tph" || key === "depth_start" || key === "depth_end"
      || key === "existing_flow_tph" || key === "added_flow_tph" || key === "calculated_flow_tph") {
    return n.toLocaleString("ru-RU", { maximumFractionDigits: 1 });
  }
  if (key === "diameter" || key === "existing_diameter" || key === "required_diameter" || key === "variant_id") {
    return String(Math.round(n));
  }
  return raw;
}

function bindGuide() {
  const guide = document.getElementById("guide");
  if (!guide) return;
  let opener = null;
  const open = (from) => {
    opener = from || document.getElementById("btn-help");
    guide.hidden = false;
    const card = guide.querySelector(".guide-card");
    if (card) card.scrollTop = 0;
    const closeBtn = document.getElementById("guide-close");
    if (closeBtn) closeBtn.focus();
  };
  const close = () => {
    guide.hidden = true;
    if (opener && opener.focus) opener.focus();
  };
  document.getElementById("btn-help").onclick = () => open(document.getElementById("btn-help"));
  const fromLegend = document.getElementById("btn-help-legend");
  if (fromLegend) fromLegend.onclick = () => open(fromLegend);
  document.getElementById("guide-close").onclick = close;
  guide.addEventListener("click", (e) => {
    if (e.target === guide) close();
  });
  document.addEventListener("keydown", (e) => {
    if (e.key === "Escape" && !guide.hidden) close();
  });
  guide.querySelectorAll(".guide-toc a").forEach((a) => {
    a.addEventListener("click", (e) => {
      const id = (a.getAttribute("href") || "").slice(1);
      const target = id && document.getElementById(id);
      if (!target) return;
      e.preventDefault();
      target.scrollIntoView({ behavior: "smooth", block: "start" });
    });
  });
}

function bindLegend() {
  const btn = document.getElementById("btn-legend-toggle");
  const legend = document.getElementById("legend");
  if (!btn || !legend) return;
  btn.onclick = () => {
    const collapsed = legend.classList.toggle("collapsed");
    btn.textContent = collapsed ? "Показать знаки" : "Свернуть";
    btn.setAttribute("aria-expanded", collapsed ? "false" : "true");
  };
}

function setStatus(id, text, kind) {
  const el = document.getElementById(id);
  el.textContent = text || "";
  el.classList.remove("ok", "err", "busy");
  if (kind) el.classList.add(kind);
}

function setDropzone(kind, title, sub) {
  const zone = document.getElementById("dropzone");
  zone.classList.remove("ok", "err", "busy", "drag");
  if (kind) zone.classList.add(kind);
  if (title != null) document.getElementById("file-label").textContent = title;
  if (sub != null) document.getElementById("file-sub").textContent = sub;
}

function setRunEnabled(on, reason) {
  state.ready = !!on;
  const btn = document.getElementById("btn-run");
  btn.disabled = !on || state.busy;
  btn.title = on
    ? "Запустить расчёт по загруженному файлу"
    : (reason || "Сначала загрузите GeoJSON");
}

function onFilePicked() {
  const file = document.getElementById("file").files[0];
  if (!file) return;
  setDropzone("busy", file.name, "Отправляю файл на сервер…");
  uploadFile(file);
}

async function runDemo() {
  try {
    setRunEnabled(false, "Дождитесь окончания загрузки");
    setDropzone("busy", "Демо-набор", "Готовлю мини-набор…");
    setStatus("upload-status", "Запускаю демо-набор...", "busy");
    const job = await api("/api/v1/demo/run?mode=" + mode(), { method: "POST" });
    await afterDatasetReady(job.datasetId, "Демо-набор загружен");
    state.jobId = job.id;
    watchJob(job.id);
  } catch (e) {
    failUpload(e);
  }
}

async function runContest() {
  try {
    setRunEnabled(false, "Дождитесь окончания загрузки");
    setDropzone("busy", "Конкурсный набор", "Готовлю конкурсный GeoJSON…");
    setStatus("upload-status", "Запускаю конкурсный набор...", "busy");
    const job = await api("/api/v1/demo/contest?mode=" + mode(), { method: "POST" });
    await afterDatasetReady(job.datasetId, "Конкурсный набор загружен");
    state.jobId = job.id;
    watchJob(job.id);
  } catch (e) {
    failUpload(e);
  }
}

async function uploadFile(picked) {
  const file = picked || document.getElementById("file").files[0];
  if (!file) {
    setStatus("upload-status", "Сначала выберите файл GeoJSON", "err");
    setRunEnabled(false, "Сначала загрузите GeoJSON");
    return;
  }
  const gen = ++state.uploadGen;
  setRunEnabled(false, "Дождитесь окончания загрузки");
  try {
    setStatus("upload-status", "Загружаю «" + file.name + "» на сервер…", "busy");
    const dataset = await postDataset(file);
    if (gen !== state.uploadGen) return;
    state.datasetId = dataset.id;
    const ready = await waitDataset(dataset.id, gen);
    if (gen !== state.uploadGen) return;
    await afterDatasetReady(ready.id, ready);
  } catch (e) {
    if (gen !== state.uploadGen) return;
    failUpload(e);
  } finally {
    const input = document.getElementById("file");
    input.value = "";
  }
}

function postDataset(file) {
  return new Promise((resolve, reject) => {
    const xhr = new XMLHttpRequest();
    xhr.open("POST", "/api/v1/datasets");
    xhr.timeout = 30 * 60 * 1000;
    xhr.upload.onprogress = (e) => {
      if (!e.lengthComputable) return;
      const pct = Math.round((100 * e.loaded) / e.total);
      setStatus("upload-status", "Загружаю «" + file.name + "»… " + pct + "%", "busy");
      setDropzone("busy", file.name, "Загрузка " + pct + "%");
    };
    xhr.onload = () => {
      let data = {};
      try { data = JSON.parse(xhr.responseText || "{}"); } catch (err) { data = {}; }
      if (xhr.status >= 200 && xhr.status < 300) {
        resolve(data);
        return;
      }
      reject(new Error(data.details || data.error || ("HTTP " + xhr.status)));
    };
    xhr.onerror = () => reject(new Error("Сеть недоступна, файл не ушёл на сервер"));
    xhr.ontimeout = () => reject(new Error("Сервер слишком долго принимал файл"));
    const body = new FormData();
    body.append("file", file, file.name);
    xhr.send(body);
  });
}

async function afterDatasetReady(datasetId, info) {
  state.datasetId = datasetId;
  const dataset = typeof info === "object" && info ? info : await api("/api/v1/datasets/" + datasetId);
  const name = dataset.originalFilename || (typeof info === "string" ? info : "набор");
  const count = dataset.featureCount || 0;
  const summary = "Файл загружен: " + name + (count ? " · объектов: " + count : "");
  setDropzone("ok", name, "Готово · объектов: " + (count || "—"));
  setStatus("upload-status", summary + ". Можно строить варианты.", "ok");
  setRunEnabled(true);
  if (!state.busy) {
    setStatus("job-status", "Нажмите «Построить варианты», чтобы начать расчёт.");
  }
  try {
    await loadInput();
  } catch (e) {
    setStatus("upload-status", summary + ". Предпросмотр не построен: " + errText(e), "ok");
  }
}

function failUpload(e) {
  state.datasetId = null;
  setRunEnabled(false, "Загрузка не удалась");
  setDropzone("err", "Загрузка не удалась", errText(e));
  setStatus("upload-status", "Ошибка загрузки: " + errText(e), "err");
}

async function waitDataset(id, gen) {
  for (let i = 0; i < 240; i++) {
    if (gen != null && gen !== state.uploadGen) throw new Error("загрузка отменена");
    const d = await api("/api/v1/datasets/" + id);
    if (d.status === "PARSED") return d;
    if (d.status === "FAILED") throw new Error(d.message || "Не удалось разобрать файл");
    const label = d.status === "PARSING" || d.status === "UPLOADED"
      ? "Разбираю GeoJSON…"
      : (d.message || d.status);
    setStatus("upload-status", label, "busy");
    setDropzone("busy", document.getElementById("file-label").textContent, label);
    await sleep(500);
  }
  throw new Error("Разбор слишком долгий");
}

async function runJob() {
  if (!state.ready || !state.datasetId) {
    setStatus("job-status", "Сначала дождитесь успешной загрузки файла", "err");
    return;
  }
  if (state.busy) return;
  state.busy = true;
  setRunEnabled(true);
  document.getElementById("bar").style.width = "0%";
  setStatus("job-status", "Запускаю расчёт…", "busy");
  try {
    const job = await api("/api/v1/jobs", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ datasetId: state.datasetId, mode: mode() }),
    });
    state.jobId = job.id;
    watchJob(job.id);
  } catch (e) {
    state.busy = false;
    setRunEnabled(true);
    setStatus("job-status", "Не удалось запустить расчёт: " + errText(e), "err");
  }
}

function watchJob(id) {
  if (state.poll) clearInterval(state.poll);
  state.busy = true;
  setRunEnabled(state.ready);
  const started = Date.now();
  let inflight = false;
  const tick = async () => {
    if (inflight) return;
    inflight = true;
    try {
      const job = await api("/api/v1/jobs/" + id);
      const sec = Math.max(0, Math.round((Date.now() - started) / 1000));
      const label = (job.progress || 0) + "% · " + (job.message || job.status)
        + (job.status === "COMPLETED" || job.status === "FAILED" ? "" : " · " + sec + " с");
      setStatus("job-status", label, job.status === "FAILED" ? "err" : "busy");
      document.getElementById("bar").style.width = (job.progress || 0) + "%";
      if (job.status === "COMPLETED") {
        clearInterval(state.poll);
        state.poll = null;
        state.busy = false;
        setRunEnabled(true);
        await loadVariants(id);
      }
      if (job.status === "FAILED") {
        clearInterval(state.poll);
        state.poll = null;
        state.busy = false;
        setRunEnabled(true);
        setStatus("job-status", "Ошибка расчёта: " + (job.error || job.message), "err");
      }
    } catch (e) {
      setStatus("job-status", "Ошибка опроса: " + errText(e), "err");
    } finally {
      inflight = false;
    }
  };
  tick();
  state.poll = setInterval(tick, 700);
}

async function loadInput() {
  if (!state.datasetId || !layers.input) return;
  const res = await fetch("/api/v1/datasets/" + state.datasetId + "/preview.geojson");
  if (!res.ok) throw new Error("сервер не отдал предпросмотр");
  const geo = await res.json();
  layers.input.clearLayers();
  layers.input.addData(geo);
  fit();
}

async function loadVariants(jobId) {
  const variants = await api("/api/v1/jobs/" + jobId + "/variants");
  state.variants = variants;
  const box = document.getElementById("variants");
  box.innerHTML = "";
  variants.forEach((v) => {
    const el = document.createElement("div");
    el.className = "card";
    const bd = v.breakdown || {};
    const tie = Number(bd.tie_in_cost || 0);
    const taps = tie > 0 ? Math.round(tie / 5000000) : 0;
    const desc = cleanText(v.description);
    const missing = v.unconnectedCount
      ? "Не подключено: " + escapeHtml((v.unconnectedIds || []).join(", "))
      : "Все ОКС подключены";
    el.innerHTML = `<div class="card-head">
        <span class="rank">${v.rank}</span>
        <div><b>${escapeHtml(cleanText(v.title) || "Вариант")}</b></div>
      </div>
      ${desc ? `<p class="card-desc">${escapeHtml(desc)}</p>` : ""}
      <dl class="metrics">
        <div><dt>Стоимость</dt><dd>${money(v.cost)}</dd></div>
        <div><dt>Длина</dt><dd>${Math.round(Number(v.lengthM) || 0).toLocaleString("ru-RU")} м</dd></div>
        <div><dt>Врезки</dt><dd>${taps}</dd></div>
        <div><dt>Камеры</dt><dd>${money(bd.chamber_construction_cost || 0)}</dd></div>
        <div><dt>Реконструкция</dt><dd>${money((Number(bd.reconstruction_cost) || 0) + (Number(bd.chamber_reconstruction_cost) || 0))}</dd></div>
        <div><dt>Рейтинг</dt><dd>${scoreText(v.score)}</dd></div>
      </dl>
      <p class="card-note${v.unconnectedCount ? " warn" : ""}">${missing}</p>`;
    el.onclick = () => selectVariant(v, el);
    box.appendChild(el);
  });
  if (variants[0]) selectVariant(variants[0], box.firstChild);
  else box.innerHTML = "<p class=\"hint\">Расчёт завершился без вариантов.</p>";
  setStatus("job-status", variants.length ? ("Готово, вариантов: " + variants.length) : "Расчёт завершён без вариантов", "ok");
}

async function selectVariant(v, el) {
  document.querySelectorAll(".card").forEach((c) => c.classList.remove("active"));
  el.classList.add("active");
  const dl = document.getElementById("download");
  dl.href = `/api/v1/jobs/${state.jobId}/variants/${v.rank}/geojson`;
  dl.classList.remove("hidden");
  const all = document.getElementById("download-all");
  all.href = `/api/v1/jobs/${state.jobId}/result.geojson`;
  all.classList.remove("hidden");
  const geo = await fetch(dl.href).then((r) => r.json());
  layers.new.clearLayers();
  layers.tap.clearLayers();
  layers.recon.clearLayers();
  const rest = { type: "FeatureCollection", features: [] };
  geo.features.forEach((f) => {
    const t = kindOf(f);
    if (!f.geometry || t === "variant_summary") return;
    if (t === "tie_in" || t === "tap_point" || t === "heat_chamber" || t === "new_chamber" || t === "technical_node") {
      L.geoJSON(f, { pointToLayer, onEachFeature }).addTo(layers.tap);
    } else if (t === "heat_network_reconstruction" || t === "heat_chamber_reconstruction" || String(t).startsWith("reconstruction")) {
      L.geoJSON(f, { style: styleResult, pointToLayer, onEachFeature }).addTo(layers.recon);
    } else {
      rest.features.push(f);
    }
  });
  layers.new.addData(rest);
  fit();
}

function fit() {
  if (!map || typeof L === "undefined") return;
  try {
    const all = L.featureGroup([layers.input, layers.new, layers.tap, layers.recon]);
    if (all.getLayers().length) map.fitBounds(all.getBounds().pad(0.12));
  } catch (e) { /* empty */ }
}

function mode() {
  return document.getElementById("mode").value;
}

async function api(url, opts) {
  const res = await fetch(url, opts);
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(data.details || data.error || res.statusText);
  return data;
}

function errText(e) {
  return String(e && e.message ? e.message : e);
}

function fmt(n) {
  return Math.round(n).toLocaleString("ru-RU");
}

function money(n) {
  const v = Number(n) || 0;
  if (Math.abs(v) >= 1000000) {
    return (v / 1000000).toLocaleString("ru-RU", { minimumFractionDigits: 1, maximumFractionDigits: 1 }) + " млн ₽";
  }
  return fmt(v) + " ₽";
}

function scoreText(n) {
  const v = Number(n);
  if (!Number.isFinite(v)) return "—";
  return v.toLocaleString("ru-RU", { minimumFractionDigits: 1, maximumFractionDigits: 1 });
}

function cleanText(s) {
  if (s == null) return "";
  const t = String(s).trim();
  return t === "null" || t === "undefined" ? "" : t;
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, (c) => ({
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    "\"": "&quot;",
    "'": "&#39;",
  }[c]));
}

function sleep(ms) {
  return new Promise((r) => setTimeout(r, ms));
}
