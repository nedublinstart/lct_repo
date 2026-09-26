# ТеплоТрасса

Сервис автоматического построения вариантов подключения перспективных ОКС к тепловой сети.

Хакатон «Лидеры цифровой трансформации» 2026 · репозиторий команды **3kalekilct**.

ТЗ лежит в [`docs/case/TZ-teploseti.pdf`](docs/case/TZ-teploseti.pdf).

## Зачем этот репозиторий удобный

- Стек **ровно как в ТЗ**: Java 11, Spring Boot **2.6.3**, springdoc-openapi-ui **1.7.0**, PostgreSQL, docker-compose 1.29.x (файл формата 2.4).
- **Один клик до карты:** загрузка GeoJSON → расчёт → три варианта → выгрузка.
- Правила кейса в [`config/appendix.yml`](config/appendix.yml): официальные таблицы DN, стоимости, ограничения и формула рейтинга. Поменяли YAML — пересчитали, **без пересборки**.
- Имена полей GeoJSON — через алиасы. Конкурсный набор (`!!!_Датасет.geojson`) читается как есть.
- Профиль `local` работает **без Docker** (H2). Прод — PostgreSQL.
- Алгоритм: обход корпусов по таблице минимальных расстояний, выходы ИТП перпендикулярно фасаду, граф видимости и лес минимальной полной стоимости (трубы по DN расхода, камеры, врезки, реконструкция). Три режима: минимальная стоимость, минимум врезок, минимум реконструкции. Подробно: [`docs/ALGORITHM.md`](docs/ALGORITHM.md).
- Готовый прогон конкурсного набора: [`samples/contest-result.geojson`](samples/contest-result.geojson) (`make contest`).

## Быстрый старт без Docker

Нужен JDK **11** (не 17/21 — требование конкурса).

```bash
export JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64   # путь может отличаться
./mvnw spring-boot:run -Dspring-boot.run.profiles=local
```

Открыть:

- Карта и загрузка: http://localhost:8080 — кнопка **«Справка»** в шапке объясняет, где какой знак и поле. Тот же текст: [`docs/SCREEN.md`](docs/SCREEN.md).
- Swagger: http://localhost:8080/swagger-ui.html
- Свой GeoJSON загружается в рамку. Перед расчётом отмечаются режимы: минимальная стоимость, минимум врезок, минимум реконструкции.

Или так:

```bash
curl -X POST http://localhost:8080/api/v1/demo/run
curl -X POST http://localhost:8080/api/v1/demo/contest
java -jar target/heatnet.jar --process-contest --out samples/contest-result.geojson
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
| GET | `/api/v1/jobs/{id}/variants/{rank}/geojson` | выгрузка одного варианта |
| GET | `/api/v1/jobs/{id}/result.geojson` | все варианты одним файлом (сдача) |
| POST | `/api/v1/demo/run` | встроенный мини-набор |
| POST | `/api/v1/demo/contest` | конкурсный `!!!_Датасет.geojson` |
| GET | `/api/v1/appendix` | текущие расчётные таблицы |

## Где что лежит

```
config/appendix.yml     ← официальные таблицы DN, стоимости, ограничения
!!!_Датасет.geojson     ← конкурсный вход
samples/contest-result.geojson ← объединённый результат (после make contest)
src/.../engine/flow      ← граф видимости, лес, смета и поиск минимальной стоимости
src/.../engine/steiner   ← прежний каркас улиц (в расчёте не используется)
src/.../costing         ← DN, реконструкция, ranking 70/30
src/.../engine/depth    ← доп. задача по Z
docs/SCREEN.md          ← справка экрана: шапка, шаги, знаки, карточки, выгрузка
docs/                   ← архитектура, алгоритм, демо, сдача
```

## Тесты

```bash
./mvnw test
```

## Важно

Числа в YAML взяты из официального технического приложения. ID и координаты конкурсного набора в алгоритм не зашиты: проверка будет на другом файле той же структуры.
