# ТеплоТрасса

Сервис автоматического построения вариантов подключения перспективных ОКС к тепловой сети.

Хакатон «Лидеры цифровой трансформации» 2026 · репозиторий команды **3kalekilct**.

ТЗ лежит в [`docs/case/TZ-teploseti.pdf`](docs/case/TZ-teploseti.pdf).

## Зачем этот репозиторий удобный

- Стек **ровно как в ТЗ**: Java 11, Spring Boot **2.6.3**, springdoc-openapi-ui **1.7.0**, PostgreSQL, docker-compose 1.29.x (файл формата 2.4).
- **Один клик до карты:** загрузка GeoJSON → расчёт → три варианта → выгрузка.
- Правила кейса в [`config/appendix.yml`](config/appendix.yml): поменяли таблицы — пересчитали, **без пересборки**.
- Имена полей GeoJSON — через алиасы, а не хардкод. Когда приедет официальное приложение, правите YAML.
- Профиль `local` работает **без Docker** (H2). Прод — PostgreSQL.
- Алгоритм уже строит сеть на мини-наборе: раздельно, совместно (MST), врезка в камеры. Это задел под промежуточную сдачу.
- Команда не пересекается: см. [`docs/TEAM.md`](docs/TEAM.md).

## Быстрый старт без Docker

Нужен JDK **11** (не 17/21 — требование конкурса).

```bash
export JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64   # путь может отличаться
./mvnw spring-boot:run -Dspring-boot.run.profiles=local
```

Открыть:

- Карта и загрузка: http://localhost:8080
- Swagger: http://localhost:8080/swagger-ui.html
- Кнопка **«Демо на мини-наборе»** прогоняет `samples/mini-input.geojson`

Или так:

```bash
curl -X POST http://localhost:8080/api/v1/demo/run
```

## Официальный стек (Ubuntu 22 + docker-compose 1.29.2)

```bash
docker-compose up --build
```

Сервис: http://localhost:8080  
PostgreSQL: `localhost:5432`, user/pass/db `heatnet`.

Лимит памяти в compose: приложение 12 ГБ, база 2 ГБ — под машину 16 ГБ из ТЗ.

## API коротко

| Метод | Путь | Зачем |
|---|---|---|
| POST | `/api/v1/datasets` | multipart `file` — входной GeoJSON до 3 ГБ |
| GET | `/api/v1/datasets/{id}` | статус разбора |
| GET | `/api/v1/datasets/{id}/preview.geojson` | исходник на карту |
| POST | `/api/v1/jobs` | `{ "datasetId", "mode": "PLAN_2D" \| "DEPTH" }` |
| GET | `/api/v1/jobs/{id}` | прогресс |
| GET | `/api/v1/jobs/{id}/variants` | рейтинг и стоимость |
| GET | `/api/v1/jobs/{id}/variants/{rank}/geojson` | выгрузка результата |
| POST | `/api/v1/demo/run` | встроенный мини-набор |
| GET | `/api/v1/appendix` | текущие расчётные таблицы |

## Где что лежит

```
config/appendix.yml     ← таблицы DN, стоимости, ограничения (править сюда)
samples/                ← крошечный GeoJSON для разработки
src/.../engine/greedy   ← поиск трасс, его и улучшаем
src/.../costing         ← DN, реконструкция, ranking 70/30
src/.../engine/depth    ← доп. задача по Z
docs/                   ← архитектура, алгоритм, демо, сдача
```

## Тесты

```bash
./mvnw test
```

## Важно

Числа в YAML — **плейсхолдеры**, пока нет официального технического приложения. Не показывайте их экспертам как «норматив».

Координаты и ID конкурсного набора в код не зашивать: проверка будет на другом файле той же структуры.
