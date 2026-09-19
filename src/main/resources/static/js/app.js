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

const state = { datasetId: null, jobId: null, poll: null, variants: [] };

document.getElementById("btn-demo").onclick = runDemo;
document.getElementById("btn-upload").onclick = uploadFile;
document.getElementById("btn-run").onclick = runJob;
document.querySelectorAll("input[data-layer]").forEach((el) => {
  el.addEventListener("change", () => {
    const layer = layers[el.dataset.layer];
    if (el.checked) map.addLayer(layer);
    else map.removeLayer(layer);
  });
});

function styleInput(feature) {
  const t = (feature.properties || {}).feature_type || "";
  if (t === "existing_segment") return { color: "#5c6370", weight: 4 };
  if (t === "oks_prospective") return { color: "#2a9d8f", weight: 2, fillOpacity: 0.25 };
  if (t === "constraint" || t === "oks_existing") return { color: "#8b6914", weight: 1, fillColor: "#c4a574", fillOpacity: 0.35 };
  return { color: "#6b5848", weight: 1, fillOpacity: 0.2 };
}

function styleResult(feature) {
  const t = (feature.properties || {}).feature_type || "";
  if (t === "new_segment") return { color: "#d04a1a", weight: 5 };
  if (t === "reconstruction_segment") return { color: "#c9a227", weight: 4, dashArray: "8 6" };
  return { color: "#1d3557", weight: 2 };
}

function pointToLayer(feature, latlng) {
  const t = (feature.properties || {}).feature_type || "";
  const colors = {
    chamber: "#1d3557",
    source: "#7a1f0d",
    connection_point: "#2a9d8f",
    tap_point: "#d04a1a",
    new_chamber: "#9b2226",
    reconstruction_chamber: "#c9a227",
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

function setStatus(id, text) {
  document.getElementById(id).textContent = text || "";
}

async function runDemo() {
  setStatus("upload-status", "Запускаю демо-набор...");
  const job = await api("/api/v1/demo/run?mode=" + mode(), { method: "POST" });
  state.datasetId = job.datasetId;
  state.jobId = job.id;
  await loadInput();
  watchJob(job.id);
}

async function uploadFile() {
  const file = document.getElementById("file").files[0];
  if (!file) {
    setStatus("upload-status", "Выберите файл");
    return;
  }
  const body = new FormData();
  body.append("file", file);
  setStatus("upload-status", "Загрузка на диск...");
  const dataset = await api("/api/v1/datasets", { method: "POST", body });
  state.datasetId = dataset.id;
  setStatus("upload-status", "Разбор " + dataset.status);
  await waitDataset(dataset.id);
  await loadInput();
  setStatus("upload-status", "Набор готов: " + dataset.originalFilename);
}

async function waitDataset(id) {
  for (let i = 0; i < 120; i++) {
    const d = await api("/api/v1/datasets/" + id);
    setStatus("upload-status", d.message || d.status);
    if (d.status === "PARSED") return d;
    if (d.status === "FAILED") throw new Error(d.message);
    await sleep(500);
  }
  throw new Error("Разбор слишком долгий");
}

async function runJob() {
  if (!state.datasetId) {
    setStatus("job-status", "Сначала загрузите набор или нажмите демо");
    return;
  }
  const job = await api("/api/v1/jobs", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ datasetId: state.datasetId, mode: mode() }),
  });
  state.jobId = job.id;
  watchJob(job.id);
}

function watchJob(id) {
  if (state.poll) clearInterval(state.poll);
  state.poll = setInterval(async () => {
    const job = await api("/api/v1/jobs/" + id);
    setStatus("job-status", (job.progress || 0) + "% · " + (job.message || job.status));
    document.getElementById("bar").style.width = (job.progress || 0) + "%";
    if (job.status === "COMPLETED") {
      clearInterval(state.poll);
      await loadVariants(id);
    }
    if (job.status === "FAILED") {
      clearInterval(state.poll);
      setStatus("job-status", "Ошибка: " + job.error);
    }
  }, 700);
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
}

async function selectVariant(v, el) {
  document.querySelectorAll(".card").forEach((c) => c.classList.remove("active"));
  el.classList.add("active");
  const dl = document.getElementById("download");
  dl.href = `/api/v1/jobs/${state.jobId}/variants/${v.rank}/geojson`;
  dl.classList.remove("hidden");
  const geo = await fetch(dl.href).then((r) => r.json());
  layers.new.clearLayers();
  layers.tap.clearLayers();
  layers.recon.clearLayers();
  const rest = { type: "FeatureCollection", features: [] };
  geo.features.forEach((f) => {
    const t = (f.properties || {}).feature_type;
    if (t === "tap_point" || t === "new_chamber") {
      L.geoJSON(f, { pointToLayer, onEachFeature }).addTo(layers.tap);
    } else if (String(t).startsWith("reconstruction")) {
      L.geoJSON(f, { style: styleResult, pointToLayer, onEachFeature }).addTo(layers.recon);
    } else if (f.geometry) {
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

function fmt(n) {
  return Math.round(n).toLocaleString("ru-RU");
}

function sleep(ms) {
  return new Promise((r) => setTimeout(r, ms));
}

window.addEventListener("unhandledrejection", (e) => {
  setStatus("job-status", String(e.reason && e.reason.message ? e.reason.message : e.reason));
});
