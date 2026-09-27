# ТеплоТрасса

Сервис строит варианты подключения новых ОКС к тепловой сети. ЛЦТ 2026, команда 3kalekilct.

На вход — один GeoJSON: существующая сеть, точки подключения, ограничения. На выход — один GeoJSON: до трёх вариантов трассы, диаметры, стоимость и показатель S. Проверка идёт по этому файлу. Карта нужна, чтобы смотреть результат, в показатель S она не входит.

Считает сервис по `config/appendix.yml`. Это таблицы технического приложения от 26.09.2026: DN, отступы, камеры, Kспец, Kгл, формула S. Постановка задачи — `docs/case/TZ-teploseti.pdf`. Координаты конкурсного набора в код не зашиты.

## Показатель S

Меньше — лучше. C — рубли (`calculated_cost`), L — метры новой сети (`new_network_length`).

```
S = 0,7 · (C / 25 000 000) + 0,3 · (L / 100)
```

В C входят:

- новая труба: длина · цена метра нового строительства · Kспец · Kгл;
- новая камера: 3 / 5 / 8 / 12 млн ₽ по DN, присоединение уже включено;
- врезка в существующую камеру: 5 млн ₽ за каждый новый участок, который в ней заканчивается;
- штраф неподключённого ОКС: 100 000 000 + 500 000 · G, где G — расход, т/ч.

Реконструкция существующей сети в C и L не входит.

В плоском режиме `PLAN_2D` коэффициент Kгл = 1, поля `depth_start` и `depth_end` равны null. Режим `DEPTH` — отдельный запуск: план тот же, затем на него кладётся профиль. Глубина считается до верха габарита. Обычная отметка 3,0 м. Глубже неё Kгл = 1 + 0,10 · (h − 3); на уклоне берётся среднее по концам участка. Координата Z в геометрию не пишется.

В файл `score` попадает с тремя знаками после запятой. Порядок вариантов считается по S до этого округления. Подпись «Минимальная стоимость» стоит на варианте с наименьшим S.

## Что лежит в выходном файле

Один FeatureCollection, геометрия WGS 84, координата — долгота и широта. Объекты: `heat_network`, `heat_chamber`, `technical_node`, `variant_summary`. У каждого объекта `variant_id` — строка `"1"`, `"2"` или `"3"`.

| Файл | Откуда | Содержимое |
|---|---|---|
| `GET /api/v1/jobs/{id}/result.geojson` | любой завершённый расчёт | все варианты этой задачи |
| `GET /api/v1/jobs/{id}/variants/1/geojson` | то же | только вариант 1, то же для 2 и 3 |
| `samples/contest-result.geojson` | офлайн-прогон, уже в репозитории | конкурсный набор, бюджет поиска 20 с |

Плоский расчёт и расчёт с глубиной — две разные задачи. Файл первой вторая не затирает. На карту для просмотра кладётся один вариант: в общем файле трассы лежат друг на друге.

Снимок конкурсного набора в `samples/contest-result.geojson`, плоский режим, 17 из 17 ОКС, врезок в существующие камеры нет:

| variant_id | Подпись | C, ₽ | L, м | S |
|---|---|---:|---:|---:|
| 1 | Минимальная стоимость | 248 546 610 | 1784,1 | 12,312 |
| 2 | Минимум врезок | 252 093 904 | 1831,7 | 12,554 |

Живой запуск с бюджетом по умолчанию 12 с может сойтись к другому лесу: поиск ограничен по времени. Образец в репозитории снят так:

```bash
java -Dheatnet.flow.budget-ms=20000 -jar target/heatnet.jar --process-contest --out samples/contest-result.geojson
```

## Три режима

Пустой список `strategies` в запросе считает все три. В смете у всех полная стоимость по приложению. Отличается только цель поиска.

| Код | Подпись | Что меняется в поиске |
|---|---|---|
| `mincost` | Минимальная стоимость | цель совпадает с S: к смете добавлено 107 143 ₽ за метр |
| `mintaps` | Минимум врезок | каждая врезка сверх первой добавляет 50 млн ₽ в цель поиска, в смету эта добавка не входит |
| `minrecon` | Короче трасса | вес длины в цели удвоен |

Почти тот же коридор второй раз не отдаётся.

## Стек

Java 11, Spring Boot 2.6.3, JTS 1.19.0, springdoc-openapi-ui 1.7.0. В контейнере PostgreSQL 16 без PostGIS, схема поднимается Hibernate. Образ: `eclipse-temurin:11-jdk` собирает jar через `./mvnw`, затем `eclipse-temurin:11-jre`. Куча расчёта в compose: `-Xmx12g`. Лимит контейнера приложения 12 ГБ, базы 2 ГБ.

Исходники: `src/main/java/ru/lct/heatnet`. Точка входа — `HeatnetApplication`. Считает `JobService`, трассу ищет `FlowRoutingEngine`.

## Как один файл проходит сервис

```
POST /api/v1/datasets
        |
        v
DatasetService          поток Jackson, объекты пачками в базу
        |
        v
POST /api/v1/jobs
        |
        v
JobService
        +-- SceneAssembler          свойства и алиасы из appendix.yml
        +-- FlowRoutingEngine       граф видимости, лес, до трёх вариантов
        +-- DiameterSelector        DN по расходу и предельной длине
        +-- DepthPostProcessor      только если mode = DEPTH
        +-- CostCalculator          трубы, камеры, врезки, штраф, Kгл
        +-- RankingCalculator       S и порядок
        +-- ResultGeoJsonExporter   один FeatureCollection
```

Разбор и расчёт идут в пуле, HTTP их не ждёт. Статус набора: `UPLOADED`, `PARSING`, `PARSED`, `FAILED`. Статус задачи: `QUEUED`, `RUNNING`, `COMPLETED`, `FAILED`. Задачу можно создать, когда набор уже `PARSED`.

После загрузки сцена в памяти поиска — это уже разобранные объекты, не поток. Файл на диске принимается до 3 ГБ.

## 1. Linux, Docker

Основной путь. Ubuntu 22, docker-compose 1.29.x (файл `docker-compose.yml` формата 2.4) или плагин `docker compose`. Нужны свободные порты 8080 и 5432 и память хоста с запасом под лимиты 12 ГБ + 2 ГБ. JDK на хосте для этого пути не нужен: Maven работает внутри образа. Первая сборка качает зависимости и требует сеть.

Из корня репозитория, где лежат `docker-compose.yml` и `!!!_Датасет.geojson`:

```bash
docker-compose up --build -d
```

Если установлен Compose V2, та же команда без дефиса: `docker compose up --build -d`.

Дождаться ответа:

```bash
curl -fsS http://localhost:8080/actuator/health
```

В теле `"status":"UP"`. Описание методов: http://localhost:8080/api.html . Swagger: http://localhost:8080/swagger-ui.html . Карта: http://localhost:8080 .

Логи:

```bash
docker-compose logs -f app
```

Остановка, том базы сохраняется:

```bash
docker-compose down
```

Полный сброс базы: `docker-compose down -v`.

Плоский результат. Разбор асинхронный, поэтому задача создаётся после `PARSED`.

```bash
mkdir -p data
UPLOAD=$(curl -fsS -F "file=@!!!_Датасет.geojson" http://localhost:8080/api/v1/datasets)
DATASET_ID=$(printf '%s' "$UPLOAD" | sed -n 's/.*"id":"\([^"]*\)".*/\1/p')
echo "dataset $DATASET_ID"

while true; do
  DS=$(curl -fsS "http://localhost:8080/api/v1/datasets/$DATASET_ID")
  echo "$DS"
  printf '%s' "$DS" | grep -q '"status":"PARSED"' && break
  printf '%s' "$DS" | grep -q '"status":"FAILED"' && exit 1
  sleep 2
done

JOB=$(curl -fsS -H "Content-Type: application/json" \
  -d "{\"datasetId\":\"$DATASET_ID\",\"mode\":\"PLAN_2D\"}" \
  http://localhost:8080/api/v1/jobs)
JOB_ID=$(printf '%s' "$JOB" | sed -n 's/.*"id":"\([^"]*\)".*/\1/p')
echo "job $JOB_ID"

while true; do
  ST=$(curl -fsS "http://localhost:8080/api/v1/jobs/$JOB_ID")
  echo "$ST"
  printf '%s' "$ST" | grep -q '"status":"COMPLETED"' && break
  printf '%s' "$ST" | grep -q '"status":"FAILED"' && exit 1
  sleep 5
done

curl -fsS -o data/result.geojson \
  "http://localhost:8080/api/v1/jobs/$JOB_ID/result.geojson"
for v in 1 2 3; do
  curl -fsS -o "data/result-${v}.geojson" \
    "http://localhost:8080/api/v1/jobs/$JOB_ID/variants/${v}/geojson"
done
```

Если третьего варианта нет, последний `curl` вернёт 404: в зачёт идут только оставшиеся после отсева близких коридоров.

Глубина — вторая задача с тем же `datasetId` и `"mode":"DEPTH"`. Файлы писать в другие имена, например `data/result-depth.geojson` и `data/result-depth-1.geojson`. В участках `depth_start` и `depth_end` — числа, метры до верха габарита. В плоском `data/result.geojson` те же поля остаются null.

Правка `config/appendix.yml` подхватывается при следующем расчёте: файл смонтирован в контейнер. Пересборка образа нужна после изменения Java.

Тот же `docker-compose.yml` поднимается в Docker Desktop. Отдельного Windows-скрипта в репозитории нет.

## 2. Без Docker

Нужен JDK 11. Профиль `local` поднимает H2, PostgreSQL не нужен.

```bash
export JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64
./mvnw spring-boot:run -Dspring-boot.run.profiles=local
```

Дальше те же адреса и тот же `curl`, что в разделе Docker.

Офлайн, без HTTP:

```bash
make contest
```

Команда собирает jar и пишет `samples/contest-result.geojson`. Бюджет поиска по умолчанию 12 с (`-Dheatnet.flow.budget-ms`).

Тесты:

```bash
./mvnw test
```

## Документы

| Файл | Содержание |
|---|---|
| `docs/ARCHITECTURE.md` | пакеты и запуск |
| `docs/ALGORITHM.md` | поиск и конкурсный прогон |
| `docs/MODEL.md` | S, лес, граф, глубина |
| `docs/DATA-CONTRACT.md` | поля входного и выходного GeoJSON |
| `docs/SCREEN.md` | экран |
| `docs/DEMO.md` | порядок проверки на карте |
| `docs/SUBMISSION.md` | поля формы сдачи |

## Каталог

```
config/appendix.yml               таблицы, по которым идёт расчёт
!!!_Датасет.geojson               конкурсный вход
samples/contest-result.geojson    результат прогона на 20 с
src/.../engine/flow               текущий поиск
src/.../costing                   DN, камеры, врезки, Kгл, S
src/.../engine/depth              профиль DEPTH
src/.../export                    GeoJSON сдачи
docker-compose.yml                Ubuntu 22, формат 2.4
```

Пакет `engine.steiner` в текущем расчёте не используется.
