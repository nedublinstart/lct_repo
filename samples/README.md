# Мини-набор `mini-input.geojson`

Синтетический район в WGS84 (условная Москва), чтобы:

- поднять сервис без конкурсного файла;
- проверить обход здания и парка;
- пересечь дорогу;
- получить три отличающихся варианта.

Объекты:

| id | type | смысл |
|---|---|---|
| SRC-1 | source | источник |
| TK-1, TK-2 | chamber | существующие камеры |
| S-1..S-3 | existing_segment | существующая сеть с `dn`, `existing_flow`, `next_id` к источнику |
| OKS-A, OKS-B | oks_prospective | два перспективных ОКС с `design_flow` |
| CP-A, CP-B | connection_point | точки на границе ОКС |
| BLD-1 | oks_existing | здание-препятствие |
| PARK-1 | constraint / park | сквер, обход |
| ROAD-1 | constraint / road | дорога, пересечение |

Когда появится официальный конкурсный набор — положите его сюда (файл не коммитьте, если он большой) и загрузите через UI или:

```bash
curl -F "file=@samples/contest.geojson" http://localhost:8080/api/v1/datasets
```
