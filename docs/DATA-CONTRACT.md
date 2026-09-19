# Контракт данных

Имена полей и значения `object_type` заданы официальным техническим приложением и продублированы в `config/appendix.yml` (алиасы + таблицы). Код не зашивает ID и координаты конкурсного набора.

Расчётный CRS: **EPSG:32637**. Ввод/вывод GeoJSON: **WGS 84**.

## Вход: один FeatureCollection

| object_type | геометрия | важные свойства |
|---|---|---|
| `source` | Point | id, name |
| `heat_network` | LineString | id, `diameter`; опционально `flow_tph`, `upstream_object_id` |
| `heat_chamber` | Point | id; опционально `upstream_object_id` |
| `oks_future` | Polygon | id, расход |
| `oks_connection_point` | Point | id, `flow_tph`; опционально `oks_id` |
| `oks_existing` | Polygon | id |
| `restriction` | Polygon | id, `restriction_type`: `oks` / `water` / `railway` / `road` / `tdtp` / `tram_tracks` / … |

Если во входе нет `oks_future`, точка подключения с расходом трактуется как перспективный ОКС. Если нет `upstream_object_id`, цепочка к источнику восстанавливается по геометрии.

Конкурсный файл в корне ветки: `!!!_Датасет.geojson` (144 объекта: 17 точек подключения, 29 участков, 9 камер, 1 источник, 88 ограничений).

## Выход: один FeatureCollection на сдачу

Свойство фичи `variant_id` ∈ {1, 2, 3}. Сводка варианта — фича без геометрии.

| object_type | смысл |
|---|---|
| `heat_network` | новая труба: `start_node_id`, `end_node_id`, `flow_tph`, `diameter`, `length`, `laying_method` (`base`\|`special`), `depth_start`/`depth_end` (null в 2D), `cost` |
| `tie_in` | врезка: `existing_object_id`, `existing_object_type`, `existing_diameter`, `required_diameter`, `cost` |
| `heat_network_reconstruction` | существующий участок, DN не хватает |
| `heat_chamber` | новая камера |
| `heat_chamber_reconstruction` | существующая камера под больший DN |
| `technical_node` | вспомогательный узел |
| `variant_summary` | `calculated_cost`, `new_network_length`, `reconstruction_length`, `score`, `unconnected_oks_ids`, разбивка стоимостей |

Скачать все варианты: `GET /api/v1/jobs/{id}/result.geojson`.  
Офлайн-файл сдачи: `samples/contest-result.geojson`.
