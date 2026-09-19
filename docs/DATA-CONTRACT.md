# Контракт данных

Официальный атрибутивный состав ещё может приехать в техническом приложении. Код **не завязан на конкретные имена полей**: они перечислены в `config/appendix.yml`.

## Вход: один FeatureCollection

Каждый Feature:

```json
{
  "type": "Feature",
  "id": "S-1",
  "properties": {
    "feature_type": "existing_segment",
    "dn": 200,
    "existing_flow": 25,
    "next_id": "TK-1"
  },
  "geometry": { "type": "LineString", "coordinates": [[37.58, 55.74], [37.581, 55.741]] }
}
```

Ожидаемые `feature_type` (и синонимы в YAML):

| kind | геометрия | важные свойства |
|---|---|---|
| existing_segment | LineString | id, dn, existing_flow, next_id |
| chamber | Point | id, next_id |
| source | Point | id |
| oks_prospective | Polygon | id, design_flow / heat_load |
| connection_point | Point | oks_id |
| constraint / oks_existing | Polygon/Line | constraint_type |

`next_id` — следующий объект **к источнику**. Цепочка может идти через участки и камеры.

Если CRS похож на WGS84, расчёт идёт в местных метрах, выгрузка — снова в lon/lat.

## Выход: один FeatureCollection на вариант

Свойства коллекции: `variant`, `title`, `total_cost`, `score`, `unconnected_oks`.

Фичи:

| feature_type | смысл |
|---|---|
| new_segment | flow_tph, dn, length_m, laying_method, cost, depth_m |
| new_chamber | dn, cost, at_tap |
| tap_point | existing_object_id, extra_flow_tph |
| technical_node | reason |
| reconstruction_segment | existing_dn, required_dn, extra_flow_tph, cost |
| reconstruction_chamber | required_dn, cost |
| unconnected_oks | id ОКС без автомаршрута |

Имена выходных типов тоже можно переименовать в `appendix.export.types`, когда организаторы зафиксируют схему.
