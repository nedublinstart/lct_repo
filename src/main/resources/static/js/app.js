const map = L.map("map").setView([55.742, 37.585], 16);
L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
  maxZoom: 20,
  attribution: "&copy; OpenStreetMap",
}).addTo(map);

const layers = {
  input: L.geoJSON(null, { style: styleInput, pointToLayer, onEachFeature }).addTo(map),
  new: L.geoJSON(null, { style: styleResult, pointToLayer, onEachFeature }).addTo(map),
  tap: L.layerGroup().addTo(map),
  recon: L.layerGroup().addTo(map),
};

const state = { datasetId: null, jobId: null, poll: null, variants: [], ready: false, busy: false };

document.getElementById("btn-demo").onclick = runDemo;
document.getElementById("btn-contest").onclick = runContest;
document.getElementById("btn-run").onclick = runJob;
document.getElementById("file").addEventListener("change", onFilePicked);
document.querySelectorAll("input[data-layer]").forEach((el) => {
  el.addEventListener("change", () => {
    const layer = layers[el.dataset.layer];
    if (el.checked) map.addLayer(layer);
    else map.removeLayer(layer);
  });
});
setRunEnabled(false, "Сначала загрузите GeoJSON");

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
  const method = (feature.properties || {}).laying_method || "";
  if ((t === "heat_network" || t === "new_segment") && method === "special") {
    return { color: "#9c3412", weight: 5, dashArray: "7 5" };
  }
  if (t === "heat_network" || t === "new_segment") return { color: "#d04a1a", weight: 5 };
  if (t === "heat_network_reconstruction" || t === "reconstruction_segment") {
    return { color: "#c9a227", weight: 4, dashArray: "8 6" };
  }
  return { color: "#1d3557", weight: 2 };
}

function pointToLayer(feature, latlng) {
  const t = kindOf(feature);
  const colors = {
    heat_chamber: "#1d3557",
    chamber: "#1d3557",
    source: "#7a1f0d",
    oks_connection_point: "#2a9d8f",
    connection_point: "#2a9d8f",
    tie_in: "#d04a1a",
    tap_point: "#d04a1a",
    new_chamber: "#9b2226",
    heat_chamber_reconstruction: "#c9a227",
    reconstruction_chamber: "#c9a227",
    technical_node: "#6b5848",
  };
  return L.circleMarker(latlng, {
    radius: t === "source" ? 9 : 6,
    color: colors[t] || "#241910",
    fillOpacity: 0.9,
    weight: 2,
  });
}

function onEachFeature(feature, layer) {
  const p = feature.properties || {};
  const rows = Object.keys(p)
    .map((k) => `<div><b>${k}</b>: ${p[k]}</div>`)
    .join("");
  layer.bindPopup(`<div class="popup">${rows}</div>`);
}

function setStatus(id, text, kind) {
  const el = document.getElementById(id);
  el.textContent = text || "";
  el.classList.remove("ok", "err", "busy");
  if (kind) el.classList.add(kind);
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
  const label = document.getElementById("file-label");
  if (!file) {
    label.textContent = "Выбрать GeoJSON";
    return;
  }
  label.textContent = file.name;
  uploadFile();
}

async function runDemo() {
  try {
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
    setStatus("upload-status", "Запускаю конкурсный набор...", "busy");
    const job = await api("/api/v1/demo/contest?mode=" + mode(), { method: "POST" });
    await afterDatasetReady(job.datasetId, "Конкурсный набор загружен");
    state.jobId = job.id;
    watchJob(job.id);
  } catch (e) {
    failUpload(e);
  }
}

async function uploadFile() {
  const file = document.getElementById("file").files[0];
  if (!file) {
    setStatus("upload-status", "Сначала выберите файл GeoJSON", "err");
    setRunEnabled(false, "Сначала загрузите GeoJSON");
    return;
  }
  setRunEnabled(false, "Дождитесь окончания загрузки");
  const body = new FormData();
  body.append("file", file);
  try {
    setStatus("upload-status", "Загружаю «" + file.name + "» на сервер…", "busy");
    const dataset = await api("/api/v1/datasets", { method: "POST", body });
    state.datasetId = dataset.id;
    const ready = await waitDataset(dataset.id);
    await afterDatasetReady(ready.id, ready);
  } catch (e) {
    failUpload(e);
  }
}

async function afterDatasetReady(datasetId, info) {
  state.datasetId = datasetId;
  const dataset = typeof info === "object" && info ? info : await api("/api/v1/datasets/" + datasetId);
  const name = dataset.originalFilename || (typeof info === "string" ? info : "набор");
  const count = dataset.featureCount || 0;
  await loadInput();
  setStatus(
    "upload-status",
    "Файл загружен: " + name + (count ? " · объектов: " + count : "") + ". Можно строить варианты.",
    "ok"
  );
  setRunEnabled(true);
  setStatus("job-status", "Нажмите «Построить варианты», чтобы начать расчёт.");
}

function failUpload(e) {
  state.datasetId = null;
  setRunEnabled(false, "Загрузка не удалась");
  setStatus("upload-status", "Ошибка загрузки: " + errText(e), "err");
}

async function waitDataset(id) {
  for (let i = 0; i < 240; i++) {
    const d = await api("/api/v1/datasets/" + id);
    const label = d.status === "PARSING" || d.status === "UPLOADED"
      ? "Разбираю GeoJSON…"
      : (d.message || d.status);
    setStatus("upload-status", label, d.status === "FAILED" ? "err" : "busy");
    if (d.status === "PARSED") return d;
    if (d.status === "FAILED") throw new Error(d.message || "Не удалось разобрать файл");
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
  const tick = async () => {
    try {
      const job = await api("/api/v1/jobs/" + id);
      setStatus("job-status", (job.progress || 0) + "% · " + (job.message || job.status), "busy");
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
    }
  };
  tick();
  state.poll = setInterval(tick, 700);
}

async function loadInput() {
  const geo = await fetch("/api/v1/datasets/" + state.datasetId + "/preview.geojson").then((r) => r.json());
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
    el.innerHTML = `<b>#${v.rank} ${v.title || ""}</b>
      <small>Стоимость: ${fmt(v.cost)} ₽ · длина: ${Math.round(v.lengthM)} м · score: ${Math.round(v.score)}</small>
      <small>${v.unconnectedCount ? "Не подключено: " + v.unconnectedIds.join(", ") : "Все ОКС подключены"}</small>`;
    el.onclick = () => selectVariant(v, el);
    box.appendChild(el);
  });
  if (variants[0]) selectVariant(variants[0], box.firstChild);
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
  const all = L.featureGroup([layers.input, layers.new, layers.tap, layers.recon]);
  if (all.getLayers().length) {
    try { map.fitBounds(all.getBounds().pad(0.12)); } catch (e) { /* empty */ }
  }
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

function sleep(ms) {
  return new Promise((r) => setTimeout(r, ms));
}

window.addEventListener("unhandledrejection", (e) => {
  setStatus("job-status", String(e.reason && e.reason.message ? e.reason.message : e.reason), "err");
});
