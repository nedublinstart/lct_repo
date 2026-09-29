# ТеплоТрасса

Сервис строит варианты подключения новых ОКС к тепловой сети. ЛЦТ 2026, команда 3kalekilct.

На вход — один GeoJSON. На выход — один GeoJSON: до трёх вариантов, диаметры, стоимость и показатель S. Проверка идёт по файлу. Карта показывает уже посчитанное.

Таблицы, по которым идёт расчёт, лежат в `config/appendix.yml` (техническое приложение от 26.09.2026). Постановка — `docs/case/TZ-teploseti.pdf`, раздел 3.2. Координаты конкурсного набора в код не зашиты.

## Содержание

- [Стек организаторов](#stack)
- [Стенд в VirtualBox](#virtualbox)
- [Поднять сервис](#up)
- [Проверить, что сервис жив](#health)
- [Проверить все команды](#check)
- [Загрузить конкурсный GeoJSON](#upload-contest)
- [Загрузить свой GeoJSON](#upload-own)
- [Дождаться разбора](#wait-parse)
- [Список наборов и объекты](#datasets)
- [Плоский расчёт](#plan)
- [Расчёт с глубиной](#depth)
- [Статус задачи](#job-status)
- [Скачать все варианты](#download-all)
- [Скачать один вариант](#download-one)
- [Карточки и рейтинг](#cards)
- [Прогон без формы загрузки](#demo)
- [Таблицы приложения](#appendix)
- [Swagger и описание полей](#swagger)
- [Карта в браузере](#map)
- [Офлайн, без сервера](#offline)
- [Остановить сервис](#down)
- [Что лежит в файле результата](#file)

Команды ниже выполняются из корня репозитория, где лежат `docker-compose.yml` и `!!!_Датасет.geojson`. Сервис слушает http://localhost:8080.

<a id="stack"></a>
## Стек организаторов

Пункт 3.2 технического задания:

| Требование | Как сделано |
|---|---|
| Ubuntu Server 22 | `docker-compose.yml`, формат 2.4, команда `docker-compose` 1.29.2 |
| Java 11 | образ `eclipse-temurin:11-jdk` собирает jar, `eclipse-temurin:11-jre` его запускает. JDK на хосте для Docker не нужен |
| spring-boot-starter-parent 2.6.3 | родитель в `pom.xml` |
| PostgreSQL до версии 18 | PostgreSQL 16. OpenSearch не используется |
| springdoc-openapi-ui 1.7.0 | Swagger: http://localhost:8080/swagger-ui.html |
| Приложение и база через docker-compose | сервисы `app` и `db` |
| Вход до 3 ГБ потоком | `POST /api/v1/datasets`, файл пишется на диск и читается Jackson по объектам |
| Выход до 500 МБ | `GET /api/v1/jobs/{id}/result.geojson` |
| ОЗУ сервера 16 ГБ | контейнер приложения 12 ГБ (`-Xmx12g`), база 2 ГБ |

Профиль внутри контейнера — `prod`. Учётная запись Postgres: база, пользователь и пароль `heatnet`, порт 5432.

<a id="virtualbox"></a>
## Стенд в VirtualBox

Команды из этого файла выполняются в терминале Ubuntu Server 22.04. Окно VirtualBox на Windows их не выполняет. В PowerShell слово `curl` означает другую программу.

Образ системы: Ubuntu Server 22.04.5 LTS, файл `ubuntu-22.04.5-live-server-amd64.iso`.

https://releases.ubuntu.com/22.04/ubuntu-22.04.5-live-server-amd64.iso

Машина:

1. «Создать». Имя `heatnet`. Тип Linux, версия Ubuntu (64-bit). Указать скачанный ISO. В мастере отметить пропуск автоматической установки, чтобы установщик Ubuntu спросил про OpenSSH сам.
2. Память виртуальной машины: 8192 МБ. Ниже 4096 МБ сборка и расчёт не рассчитаны. Процессоры: 4. Диск: 40 ГБ, динамический.
3. Сеть: адаптер NAT. «Дополнительно» → «Проброс портов»: хост `2222` на гость `22`, хост `8080` на гость `8080`.
4. Если в списке версий нет Ubuntu (64-bit), в BIOS компьютера включается виртуализация Intel VT-x или AMD-V.

Установщик Ubuntu:

- занять весь диск;
- на экране OpenSSH отметить «Install OpenSSH server»;
- на экране snaps ничего не отмечать;
- запомнить логин и пароль.

После перезагрузки войти в чёрное окно машины. Приглашение вида `nedublin@heatnet` значит, что вход уже выполнен, следующие команды набираются здесь.

Строка `New release '24.04.5 LTS'` — предложение уйти с Ubuntu 22.04. Команду `do-release-upgrade` не запускать: стек задания собран под 22.04.

Команда `ssh -p 2222 ЛОГИН@127.0.0.1` набирается на компьютере, в PowerShell или cmd, когда нужно зайти в машину снаружи. В окне, где уже есть приглашение `nedublin@heatnet`, её набирать не нужно: порт 2222 там никто не слушает, ответ будет `Connection refused`.

Пакеты стека. `docker-compose` из репозитория Ubuntu 22.04 — версия 1.29.2.

```bash
sudo apt update
sudo apt install -y docker.io docker-compose git curl
sudo usermod -aG docker $USER
```

Выйти командой `exit` и войти тем же логином ещё раз: группа `docker` подхватывается при новом входе. Проверка:

```bash
docker-compose --version
```

Ожидается строка `docker-compose version 1.29.2`.

Репозиторий открытый, токен для клона не нужен.

```bash
git clone https://github.com/nedublinstart/lct_repo.git
cd lct_repo
```

Если у машины меньше 7 ГБ памяти, лимиты контейнеров в файле больше, чем есть у машины. Перед запуском их нужно уменьшить:

```bash
mem=$(awk '/MemTotal/ {print int($2/1024/1024)}' /proc/meminfo)
echo "память машины: ${mem} ГБ"
if [ "$mem" -lt 7 ]; then
  sed -i '0,/mem_limit: 2g/s//mem_limit: 512m/' docker-compose.yml
  sed -i 's/-Xmx12g/-Xmx1536m/' docker-compose.yml
  sed -i 's/mem_limit: 12g/mem_limit: 2560m/' docker-compose.yml
fi
```

Дальше [поднять сервис](#up). Карта с компьютера открывается как http://localhost:8080. Команды `curl` выполняются в этом сеансе SSH.

<a id="up"></a>
## Поднять сервис

Нужны Docker и пакет `docker-compose` 1.29.2 (команда через дефис). Порты 8080 и 5432 свободны. На хосте запас памяти под 12 ГБ + 2 ГБ. Первая сборка качает образы и зависимости Maven, нужен выход в сеть.

```bash
docker-compose up --build -d
```

Флаг `-d` отпускает терминал. Без него тот же стек остаётся в этом окне. Если установлен только Compose V2, команда та же без дефиса: `docker compose up --build -d`.

Повторный запуск на `docker-compose` 1.29.2 иногда кончается строкой `KeyError: 'ContainerConfig'`. Тогда контейнеры создаются заново:

```bash
docker-compose down
docker-compose up -d
```

Логи приложения:

```bash
docker-compose logs -f app
```

<a id="health"></a>
## Проверить, что сервис жив

```bash
curl -fsS http://localhost:8080/actuator/health
```

Ответ: `{"status":"UP"}`. Пока Postgres поднимается, приложение ждёт его до 120 секунд и health может не отвечать. Повторите команду.

Справка по полям: http://localhost:8080/api.html  
Карта: http://localhost:8080

<a id="check"></a>
## Проверить все команды

Мини-набор уже внутри сервиса. Конкурсный файл для этой проверки не нужен. Расчёт занимает бюджет поиска: 12 секунд, если в `JAVA_OPTS` не задано иное.

```bash
curl -fsS http://localhost:8080/actuator/health; echo
curl -fsS http://localhost:8080/api/v1/appendix/meta; echo
curl -s -o /dev/null -w 'swagger %{http_code}\n' http://localhost:8080/swagger-ui.html
curl -fsS -o /dev/null -w 'api.html %{http_code}\n' http://localhost:8080/api.html

JOB=$(curl -fsS -X POST "http://localhost:8080/api/v1/demo/run?mode=PLAN_2D")
echo "$JOB"
JOB_ID=$(printf '%s' "$JOB" | sed -n 's/.*"id":"\([^"]*\)".*/\1/p')
DATASET_ID=$(printf '%s' "$JOB" | sed -n 's/.*"datasetId":"\([^"]*\)".*/\1/p')

while true; do
  ST=$(curl -fsS "http://localhost:8080/api/v1/jobs/$JOB_ID")
  echo "$ST"
  printf '%s' "$ST" | grep -q '"status":"COMPLETED"' && break
  printf '%s' "$ST" | grep -q '"status":"FAILED"' && exit 1
  sleep 2
done

curl -fsS "http://localhost:8080/api/v1/datasets/$DATASET_ID/features?kind=OKS_PROSPECTIVE&limit=50&offset=0"; echo
curl -fsS "http://localhost:8080/api/v1/jobs/$JOB_ID/variants"; echo
mkdir -p data
curl -fsS -o data/mini-result.geojson "http://localhost:8080/api/v1/jobs/$JOB_ID/result.geojson"
curl -fsS -o data/mini-v1.geojson "http://localhost:8080/api/v1/jobs/$JOB_ID/variants/1/geojson"
curl -s -o /dev/null -w 'вариант 3: HTTP %{http_code}\n' \
  "http://localhost:8080/api/v1/jobs/$JOB_ID/variants/3/geojson"
```

Ожидание: health `{"status":"UP"}`, swagger `302`, задача доходит до `COMPLETED`. Вариант 3 на мини-наборе отвечает 404: в файл попадает один коридор. Конкурсный файл — отдельно: [загрузка](#upload-contest), [разбор](#wait-parse), [план](#plan), [глубина](#depth), [скачивание](#download-all).

<a id="upload-contest"></a>
## Загрузить конкурсный GeoJSON

Файл в корне репозитория: `!!!_Датасет.geojson`. Поле формы называется `file`. Ответ приходит сразу, разбор идёт отдельно. В ответе нужен `id` набора.

Имя файла начинается с `!`. В команде оно в одинарных кавычках. В обычном терминале bash читает `!` как обращение к истории, и запрос на сервер не уходит.

```bash
mkdir -p data
UPLOAD=$(curl -fsS -F 'file=@!!!_Датасет.geojson' http://localhost:8080/api/v1/datasets)
echo "$UPLOAD"
DATASET_ID=$(printf '%s' "$UPLOAD" | sed -n 's/.*"id":"\([^"]*\)".*/\1/p')
echo "dataset $DATASET_ID"
```

Дальше [дождитесь разбора](#wait-parse), затем [плоский расчёт](#plan).

<a id="upload-own"></a>
## Загрузить свой GeoJSON

Та же команда, другое имя файла. Набор должен быть в той же структуре: существующая сеть, точки подключения, ограничения.

```bash
UPLOAD=$(curl -fsS -F 'file=@путь/к/файлу.geojson' http://localhost:8080/api/v1/datasets)
DATASET_ID=$(printf '%s' "$UPLOAD" | sed -n 's/.*"id":"\([^"]*\)".*/\1/p')
echo "dataset $DATASET_ID"
```

Лимит — 3 ГБ. Более крупный файл сервер отвергает.

<a id="wait-parse"></a>
## Дождаться разбора

Задачу можно создать только когда `status` равен `PARSED`. Раньше сервер ответит 409.

```bash
while true; do
  DS=$(curl -fsS "http://localhost:8080/api/v1/datasets/$DATASET_ID")
  echo "$DS"
  printf '%s' "$DS" | grep -q '"status":"PARSED"' && break
  printf '%s' "$DS" | grep -q '"status":"FAILED"' && exit 1
  sleep 2
done
```

Статусы набора: `UPLOADED`, `PARSING`, `PARSED`, `FAILED`.

Один запрос состояния:

```bash
curl -fsS "http://localhost:8080/api/v1/datasets/$DATASET_ID"
```

<a id="datasets"></a>
## Список наборов и объекты

Список:

```bash
curl -fsS http://localhost:8080/api/v1/datasets
```

Объекты набора. `kind` необязателен. Значения: `EXISTING_SEGMENT`, `CHAMBER`, `SOURCE`, `OKS_PROSPECTIVE`, `OKS_EXISTING`, `CONNECTION_POINT`, `CONSTRAINT`, `UNKNOWN`.

```bash
curl -fsS "http://localhost:8080/api/v1/datasets/$DATASET_ID/features?kind=OKS_PROSPECTIVE&limit=50&offset=0"
```

Предпросмотр для карты, WGS 84:

```bash
curl -fsS -o data/preview.geojson \
  "http://localhost:8080/api/v1/datasets/$DATASET_ID/preview.geojson"
```

<a id="plan"></a>
## Плоский расчёт

`mode` равен `PLAN_2D`. Пустой список режимов считает все три: минимальная стоимость, минимум врезок, минимальная длина. В смете у всех полная стоимость по приложению. Отличается цель поиска.

| Код | Подпись | Цель поиска |
|---|---|---|
| `mincost` | Минимальная стоимость | совпадает с S |
| `mintaps` | Минимум врезок | каждая врезка сверх первой добавляет 50 млн ₽ в цель, в смету эта добавка не входит |
| `minrecon` | Минимальная длина | наименьшая новая сеть; метр в цели стоит 100 млн ₽, в смету этот вес не входит |

Подпись «Минимальная стоимость» после расчёта стоит на варианте с наименьшим S. Более длинный коридор, который не лучше по врезкам, второй раз не отдаётся. Более короткая трасса остаётся, даже если её S выше.

Все три режима:

```bash
JOB=$(curl -fsS -H "Content-Type: application/json" \
  -d "{\"datasetId\":\"$DATASET_ID\",\"mode\":\"PLAN_2D\"}" \
  http://localhost:8080/api/v1/jobs)
echo "$JOB"
JOB_ID=$(printf '%s' "$JOB" | sed -n 's/.*"id":"\([^"]*\)".*/\1/p')
echo "job $JOB_ID"
```

Только стоимость и врезки, без минимальной длины:

```bash
JOB=$(curl -fsS -H "Content-Type: application/json" \
  -d "{\"datasetId\":\"$DATASET_ID\",\"mode\":\"PLAN_2D\",\"strategies\":[\"mincost\",\"mintaps\"]}" \
  http://localhost:8080/api/v1/jobs)
JOB_ID=$(printf '%s' "$JOB" | sed -n 's/.*"id":"\([^"]*\)".*/\1/p')
echo "job $JOB_ID"
```

Дальше [статус](#job-status) и [скачивание](#download-all).

Бюджет поиска по умолчанию 12 секунд на все выбранные режимы. Файл `samples/contest-result.geojson` снят с этим бюджетом. Для 20 секунд в `docker-compose.yml` у сервиса `app` строка окружения такая:

```yaml
JAVA_OPTS: -Xms512m -Xmx12g -XX:+UseG1GC -Dheatnet.flow.budget-ms=20000
```

После правки контейнер пересоздаётся без пересборки образа: `docker-compose up -d`.

<a id="depth"></a>
## Расчёт с глубиной

Отдельная задача, тот же `DATASET_ID`. Плоский файл она не затирает. План строится тем же поиском, затем на него кладётся профиль. `depth_start` и `depth_end` — метры до верха габарита. Координата Z не пишется. Глубже 3,0 м коэффициент Kгл = 1 + 0,10 · (h − 3), на уклоне берётся среднее по концам, S пересчитывается.

```bash
JOB=$(curl -fsS -H "Content-Type: application/json" \
  -d "{\"datasetId\":\"$DATASET_ID\",\"mode\":\"DEPTH\"}" \
  http://localhost:8080/api/v1/jobs)
JOB_ID=$(printf '%s' "$JOB" | sed -n 's/.*"id":"\([^"]*\)".*/\1/p')
echo "depth job $JOB_ID"
```

Дождитесь [статуса](#job-status) `COMPLETED` и сохраните файл под другим именем. Плоский `data/result.geojson` эта команда не трогает.

```bash
curl -fsS -o data/result-depth.geojson \
  "http://localhost:8080/api/v1/jobs/$JOB_ID/result.geojson"
curl -fsS -o data/result-depth-1.geojson \
  "http://localhost:8080/api/v1/jobs/$JOB_ID/variants/1/geojson"
```

<a id="job-status"></a>
## Статус задачи

Статусы: `QUEUED`, `RUNNING`, `COMPLETED`, `FAILED`. В `message` — текущий шаг, в `progress` — проценты.

```bash
while true; do
  ST=$(curl -fsS "http://localhost:8080/api/v1/jobs/$JOB_ID")
  echo "$ST"
  printf '%s' "$ST" | grep -q '"status":"COMPLETED"' && break
  printf '%s' "$ST" | grep -q '"status":"FAILED"' && exit 1
  sleep 5
done
```

Один запрос:

```bash
curl -fsS "http://localhost:8080/api/v1/jobs/$JOB_ID"
```

<a id="download-all"></a>
## Скачать все варианты

Это файл сдачи. В нём до трёх трасс, у каждой свой `variant_id`: `"1"`, `"2"`, `"3"`. Идентификатор каждого своего объекта в файле уникален и начинается с `variant_id`. Идентификаторы входных точек подключения и существующих камер в `start_node_id` и `end_node_id` пишутся как во входном файле.

```bash
curl -fsS -o data/result.geojson \
  "http://localhost:8080/api/v1/jobs/$JOB_ID/result.geojson"
```

<a id="download-one"></a>
## Скачать один вариант

На карту кладите один файл: в общем результате трассы лежат друг на друге. Ранг начинается с 1.

```bash
curl -fsS -o data/result-1.geojson \
  "http://localhost:8080/api/v1/jobs/$JOB_ID/variants/1/geojson"
curl -fsS -o data/result-2.geojson \
  "http://localhost:8080/api/v1/jobs/$JOB_ID/variants/2/geojson"
curl -fsS -o data/result-3.geojson \
  "http://localhost:8080/api/v1/jobs/$JOB_ID/variants/3/geojson"
```

Если варианта с таким рангом нет, ответ 404. Это нормально: близкий коридор в файл не попадает.

<a id="cards"></a>
## Карточки и рейтинг

Список без геометрии. Ранг 1 — наименьший S.

```bash
curl -fsS "http://localhost:8080/api/v1/jobs/$JOB_ID/variants"
```

Одна карточка:

```bash
curl -fsS "http://localhost:8080/api/v1/jobs/$JOB_ID/variants/1"
```

Поля карточки: `rank`, `title`, `code`, `cost`, `lengthM`, `score`, `unconnectedCount`, `unconnectedIds`, `breakdown`. В `breakdown` есть `pipe_cost`, `chamber_construction_cost`, `existing_chamber_tie_in_cost`, `unconnected_penalty`, `calculated_cost`. Поля реконструкции равны 0. В карточке `score` — полное число. В GeoJSON оно записано с тремя знаками.

<a id="demo"></a>
## Прогон без формы загрузки

Мини-набор встроен в сервис. Разбор заканчивается до ответа, в теле сразу задача.

```bash
curl -fsS -X POST "http://localhost:8080/api/v1/demo/run?mode=PLAN_2D"
```

Конкурсный файл ищется на сервере. `docker-compose.yml` монтирует `!!!_Датасет.geojson` из корня репозитория в `/app/!!!_Датасет.geojson`. При запуске без Docker из каталога репозитория берётся тот же файл в текущем каталоге.

```bash
curl -fsS -X POST "http://localhost:8080/api/v1/demo/contest?mode=PLAN_2D"
curl -fsS -X POST "http://localhost:8080/api/v1/demo/contest?mode=DEPTH"
```

Из ответа берётся `id` задачи и дальше те же [статус](#job-status) и [скачивание](#download-all). Если конкурсного файла на сервере нет, ответ 404.

<a id="appendix"></a>
## Таблицы приложения

Файл перечитывается при каждом запросе. После правки `config/appendix.yml` пересборка образа не нужна: каталог смонтирован в контейнер. Следующий расчёт уже видит новые таблицы. Пересборка нужна после изменения Java: `docker-compose up --build -d`.

```bash
curl -fsS http://localhost:8080/api/v1/appendix/meta
curl -fsS http://localhost:8080/api/v1/appendix/raw
curl -fsS -o data/appendix.json http://localhost:8080/api/v1/appendix
```

`meta` показывает путь к приложению, каталог данных, лимит загрузки и флаг `async`.

<a id="swagger"></a>
## Swagger и описание полей

- http://localhost:8080/swagger-ui.html — Swagger UI, адрес перенаправляет на `/swagger-ui/index.html`
- http://localhost:8080/v3/api-docs — перечень методов
- http://localhost:8080/api.html — поля GeoJSON, статусы, состав сметы

<a id="map"></a>
## Карта в браузере

Откройте http://localhost:8080. Файл уходит на сервер сразу после выбора. Кнопка «Рассчитать» включается после разбора.

Режим в списке: «План» (`PLAN_2D`) или «С глубиной» (`DEPTH`). По умолчанию отмечены все три: «Минимальная стоимость», «Минимум врезок» и «Минимальная длина». «Все варианты» скачивает общий файл, «GeoJSON варианта» — один ранг.

<a id="offline"></a>
## Офлайн, без сервера

На хосте нужен JDK 11. База не нужна.

```bash
export JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64
./mvnw -DskipTests package
java -Dheatnet.flow.budget-ms=20000 -jar target/heatnet.jar \
  --process-contest --mode PLAN_2D --out samples/contest-result.geojson
```

Глубина: `--mode DEPTH` и другой `--out`. `make contest` делает тот же прогон в режиме `PLAN_2D` с бюджетом 12 с и пишет результат в `samples/contest-result.geojson`, поверх снимка из репозитория.

Локальный сервер без Docker, база H2:

```bash
export JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64
./mvnw spring-boot:run -Dspring-boot.run.profiles=local
```

Дальше те же адреса и те же команды `curl`.

Тесты:

```bash
./mvnw test
```

<a id="down"></a>
## Остановить сервис

Том базы сохраняется:

```bash
docker-compose down
```

Сбросить базу:

```bash
docker-compose down -v
```

Каталог `data/` на хосте при этом остаётся: туда `curl -o` кладёт скачанные файлы.

<a id="file"></a>
## Что лежит в файле результата

Один FeatureCollection, имя `heatnet-result`, геометрия WGS 84, координата — долгота и широта. Объекты: `heat_network`, `heat_chamber`, `technical_node`, `variant_summary`.

```
S = 0,7 · (C / 25 000 000) + 0,3 · (L / 100)
```

Меньше — лучше. C — `calculated_cost`, рубли. L — `new_network_length`, метры новой сети. В C входят труба (длина · цена метра · Kспец · Kгл), новая камера 3 / 5 / 8 / 12 млн ₽ по DN, врезка в существующую камеру 5 млн ₽ за каждый новый участок и штраф неподключённого ОКС 100 000 000 + 500 000 · G. Реконструкция существующей сети в C и L не входит.

В режиме «План» Kгл = 1, `depth_start` и `depth_end` равны null. В файл `score` пишется с тремя знаками. Порядок вариантов считается по S до округления.

Снимок конкурсного набора в репозитории, бюджет по умолчанию 12 с, 17 из 17 ОКС, врезок в существующие камеры нет:

| variant_id | Подпись | C, ₽ | L, м | S |
|---|---|---:|---:|---:|
| 1 | Минимальная стоимость | 249 753 200 | 1771,4 | 12,307 |
| 2 | Минимум врезок | 253 326 117 | 1819,4 | 12,551 |
| 3 | Минимальная длина | 274 609 901 | 1690,9 | 12,762 |

Подробнее: `docs/DATA-CONTRACT.md`, `docs/ALGORITHM.md`, `docs/MODEL.md`, `docs/ANNOTATION.md`, `docs/ARCHITECTURE.md`.
