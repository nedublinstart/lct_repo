# ТеплоТрасса

Сервис построения вариантов подключения ОКС к тепловой сети. ЛЦТ 2026, команда 3kalekilct.

Java 11, Spring Boot 2.6.3, springdoc-openapi-ui 1.7.0. Таблицы DN, стоимости, ограничений и формула S — в `config/appendix.yml` (техническое приложение от 26.09.2026). Конкурсный вход — `!!!_Датасет.geojson`. Результат прогона — `samples/contest-result.geojson`.

Координаты и id конкурсного набора в код не зашиты.

## Запуск без Docker

Нужен JDK 11.

```bash
export JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64
./mvnw spring-boot:run -Dspring-boot.run.profiles=local
```

Профиль `local` поднимает H2. PostgreSQL и Docker не нужны.

- Карта и загрузка: http://localhost:8080
- Описание полей и методов: http://localhost:8080/api.html
- Swagger: http://localhost:8080/swagger-ui.html

В рамку загружается GeoJSON. Перед расчётом отмечаются режимы: минимальная стоимость, минимум врезок, короче трасса. Кнопка «Рассчитать». Справка по знакам карты — кнопка «Справка», тот же текст в `docs/SCREEN.md`.

Офлайн, без сервера:

```bash
make contest
```

Команда собирает jar и пишет `samples/contest-result.geojson`. Бюджет поиска по умолчанию 12 с на все выбранные режимы (`-Dheatnet.flow.budget-ms`). Файл в репозитории снят при 20 с.

Проверка методов без карты:

```bash
curl -X POST http://localhost:8080/api/v1/demo/run
curl -X POST "http://localhost:8080/api/v1/demo/contest?mode=PLAN_2D"
```

## Docker

Ubuntu 22, docker-compose 1.29.x, файл формата 2.4:

```bash
docker-compose up --build
```

Сервис: http://localhost:8080. PostgreSQL: `localhost:5432`, база, пользователь и пароль `heatnet`. Лимит памяти: приложение 12 ГБ, база 2 ГБ.

## API

| Метод | Путь | Назначение |
|---|---|---|
| POST | `/api/v1/datasets` | multipart `file`, входной GeoJSON до 3 ГБ |
| GET | `/api/v1/datasets/{id}` | статус разбора |
| GET | `/api/v1/datasets/{id}/preview.geojson` | исходник на карту |
| POST | `/api/v1/jobs` | `{ "datasetId", "mode": "PLAN_2D" \| "DEPTH", "strategies": ["mincost", "mintaps", "minrecon"] }` |
| GET | `/api/v1/jobs/{id}` | прогресс |
| GET | `/api/v1/jobs/{id}/variants` | рейтинг и стоимость |
| GET | `/api/v1/jobs/{id}/variants/{rank}/geojson` | один вариант |
| GET | `/api/v1/jobs/{id}/result.geojson` | все варианты одним файлом |
| POST | `/api/v1/demo/run?mode=PLAN_2D` | встроенный мини-набор |
| POST | `/api/v1/demo/contest?mode=PLAN_2D` | `!!!_Датасет.geojson` |
| GET | `/api/v1/appendix` | расчётные таблицы |
| GET | `/api/v1/appendix/raw` | тот же файл, YAML |
| GET | `/api/v1/appendix/meta` | путь к приложению, каталог данных, лимит загрузки |

`minrecon` в запросе — режим «Короче трасса». Состав GeoJSON — на `/api.html`.

## Документы

| Файл | Содержание |
|---|---|
| `docs/ARCHITECTURE.md` | стек, пакеты, запуск |
| `docs/ALGORITHM.md` | поиск, смета, конкурсный прогон |
| `docs/MODEL.md` | формула S, лес, граф, глубина |
| `docs/DATA-CONTRACT.md` | входной и выходной GeoJSON |
| `docs/SCREEN.md` | экран |
| `docs/DEMO.md` | порядок проверки |
| `docs/SUBMISSION.md` | поля формы сдачи |

## Каталог

```
config/appendix.yml              таблицы DN, стоимости, ограничений
!!!_Датасет.geojson              конкурсный вход
samples/contest-result.geojson   результат прогона
src/.../engine/flow              текущий расчёт
src/.../costing                  DN, камеры, врезки, Kгл, рейтинг
src/.../engine/depth             режим DEPTH
src/.../engine/steiner           прежний каркас, в расчёте не используется
```

## Тесты

```bash
./mvnw test
```
