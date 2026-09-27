# Мини-набор `mini-input.geojson`

Синтетический район в WGS84 (условная Москва), чтобы:

- поднять сервис без конкурсного файла;
- проверить обход здания и парка;
- пересечь дорогу;
- получить варианты расчёта.

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

Конкурсный вход лежит в корне ветки: `!!!_Датасет.geojson`. Готовый плоский результат — `samples/contest-result.geojson` (`make contest`). Загрузка через интерфейс или:

```bash
curl -F 'file=@!!!_Датасет.geojson' http://localhost:8080/api/v1/datasets
```
