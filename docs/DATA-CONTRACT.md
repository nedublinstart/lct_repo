# Контракт данных

Имена полей и значения `object_type` заданы техническим приложением от 26 сентября 2026 и продублированы в `config/appendix.yml`. Код не зашивает ID и координаты конкурсного набора.

Расчётный CRS: **EPSG:32637**. Ввод и вывод GeoJSON: **WGS 84**. Координата Z в геометрию не пишется.

## Вход: один FeatureCollection

| object_type | геометрия | важные свойства |
|---|---|---|
| `source` | Point | id, name |
| `heat_network` | LineString | id, `diameter`; опционально `flow_tph`, `upstream_object_id` |
| `heat_chamber` | Point | id; опционально `upstream_object_id` |
| `oks_future` | Polygon | id, расход |
| `oks_connection_point` | Point | id, `flow_tph`; опционально `oks_id` |
| `oks_existing` | Polygon | id |
| `restriction` | Polygon | id, `restriction_type`: `oks`, `water`, `railway`, `road`, `tdtp`, `tram_tracks`, `gas_pipeline`, `power_cable` и другие из таблицы приложения |

Если во входе нет `oks_future`, точка подключения с расходом считается перспективным ОКС. Если нет `upstream_object_id`, цепочка к источнику восстанавливается по геометрии в окне `routing.endpoint-snap-m` (4 м).

Дорога, трамвай, газ, кабель и существующая теплосеть учитываются, когда их полигон или линия есть во входном файле. Улицы подложки OpenStreetMap в расчёт не входят.

Конкурсный файл в корне ветки: `!!!_Датасет.geojson` (144 объекта: 17 точек подключения, 29 участков, 9 камер, 1 источник, 88 ограничений). Дорог, газа и кабеля в нём нет.

## Выход: один FeatureCollection

До трёх вариантов. `variant_id` — строка `"1"`, `"2"` или `"3"`. Сводка варианта — объект без геометрии.

| object_type | свойства |
|---|---|
| `heat_network` | `start_node_id`, `end_node_id`, `flow_tph`, `diameter`, `length`, `laying_method` (`base` или `special`), `depth_start`, `depth_end`, `cost` |
| `heat_chamber` | новая камера: `diameter`, `cost` |
| `technical_node` | стык участков, без стоимости |
| `variant_summary` | `rank`, `construction_cost`, `chamber_construction_cost`, `existing_chamber_tie_in_count`, `existing_chamber_tie_in_cost`, `unconnected_penalty`, `calculated_cost`, `new_network_length`, `score`, `unconnected_oks_ids` |

В плоском режиме `depth_start` и `depth_end` равны null. В режиме DEPTH это глубина до верха габарита в метрах, четыре знака после запятой. Длина участка — горизонтальная проекция.

Отдельных объектов врезки и реконструкции в файле нет. Врезка в существующую камеру входит в `existing_chamber_tie_in_cost`: 5 000 000 ₽ за каждый новый участок, который в ней заканчивается. Новая камера уже включает присоединение.

Скачать все варианты: `GET /api/v1/jobs/{id}/result.geojson`.  
Плоский офлайн-файл: `samples/contest-result.geojson`.
