# Архитектура

Монолит Spring Boot 2.6.3 / Java 11, чтобы команда из 2–5 человек не тратила день на микросервисы.

```
браузер  →  static UI (Leaflet)
Swagger  →  REST /api/v1
             ├─ DatasetService   потоковая загрузка GeoJSON на диск + разбор
             ├─ SceneAssembler   свойства → домен (с алиасами из YAML)
             ├─ RoutingEngine    3 варианта трасс (пакет engine.greedy)
             ├─ costing          диаметры, реконструкция, стоимость, рейтинг
             ├─ DepthPostProcessor  доп. задача, не ломает 2D
             └─ ResultGeoJsonExporter
                    ↓
            PostgreSQL / H2     метаданные наборов и задач
            ./data              исходники и результаты на диске
```

## Почему так

- **Долгий расчёт** вынесен в пул `heatnetExecutor` (2–4 потока). HTTP не блокируется, UI поллит `/jobs/{id}`.
- **Файлы до 3 ГБ** не читаются целиком: Jackson streaming + батч в БД по 400 фич, геометрия как текст.
- **Правила кейса не зашиты в Java.** `config/appendix.yml` перечитывается перед каждым расчётом. Получили официальные таблицы — правите YAML, жмёте «Построить варианты».
- **Алгоритм сменный.** `RoutingEngine` — один интерфейс. Сейчас `GreedyRoutingEngine`. Можно добавить `SteinerEngine` без изменения API.
- **Профиль `local`** поднимает H2, Docker не обязателен для разработки. Профиль `prod` — PostgreSQL 16, как в ТЗ.

## Куда класть новый код

| Задача | Пакет |
|---|---|
| Новые поля входного GeoJSON | `ingest`, `config/appendix.yml` → `aliases` |
| Другой поиск трассы | `engine` / новый класс + бин вместо greedy |
| Стоимость и DN | `costing` + YAML |
| Выходной формат | `export` + `appendix.export.types` |
| Карта / демо | `src/main/resources/static` |
| Глубина | `engine.depth` |
