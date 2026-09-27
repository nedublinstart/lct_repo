# Сдача

Поля ниже копируются в форму. Ссылки заработают после двух действий: репозиторий открыт для жюри, текущий расчёт лежит в ветке, на которую ведёт ссылка.

Репозиторий сейчас private. Ветка `main` от 19.09.2026 — прежний каркас. Текущий расчёт туда не влит. Ссылка на корень репозитория откроет жюри не тот README и не тот алгоритм.

Ссылка, в которой лежат и текущий расчёт, и эта инструкция: ветка `cursor/submission-form-258a`. После вливания в `main` кусок `/tree/cursor/submission-form-258a` и `/blob/cursor/submission-form-258a` из ссылок убирается.

## Поля формы

Репозиторий

https://github.com/nedublinstart/3kalekilct/tree/cursor/submission-form-258a

В корне `README.md`: JDK 11 и `./mvnw spring-boot:run -Dspring-boot.run.profiles=local`, либо `docker-compose up --build`. Офлайн-прогон конкурсного файла: `make contest`.

Документация

https://github.com/nedublinstart/3kalekilct/blob/cursor/submission-form-258a/docs/ARCHITECTURE.md

Стек, пакеты, запуск. Дальше по тому же дереву: `docs/ALGORITHM.md`, `docs/MODEL.md`, `docs/DATA-CONTRACT.md`. Описание методов — `src/main/resources/static/api.html`, после запуска это http://localhost:8080/api.html.

Прототип

Публичного стенда нет. Подъём, проверка команд, загрузка `!!!_Датасет.geojson` и скачивание результата — в `README.md`: [поднять сервис](../README.md#up), [проверить все команды](../README.md#check), [загрузить конкурсный файл](../README.md#upload-contest), [скачать все варианты](../README.md#download-all). После запуска:

- http://localhost:8080 — карта, загрузка, расчёт
- http://localhost:8080/api.html — поля и методы
- http://localhost:8080/swagger-ui.html — Swagger

Готовый результат конкурсного набора, бюджет 20 с: `samples/contest-result.geojson`. «Минимальная стоимость» — 248 546 610 ₽, 1784 м, S = 12,312, 17/17 ОКС. «Минимум врезок» — 252 093 904 ₽, 1832 м, S = 12,554.

Дополнительные материалы

- https://github.com/nedublinstart/3kalekilct/blob/cursor/submission-form-258a/samples/contest-result.geojson
- https://github.com/nedublinstart/3kalekilct/blob/cursor/submission-form-258a/docs/MODEL.md
- https://github.com/nedublinstart/3kalekilct/blob/cursor/submission-form-258a/docs/DEMO.md
- https://github.com/nedublinstart/3kalekilct/blob/cursor/submission-form-258a/docs/SCREEN.md
- https://github.com/nedublinstart/3kalekilct/blob/cursor/submission-form-258a/config/appendix.yml

## Промежуточная / текущее состояние

- Каркас Java 11 + Spring Boot 2.6.3 + springdoc 1.7.0 + docker-compose 2.4.
- Потоковая загрузка GeoJSON, PostgreSQL/H2, карта. Перед расчётом выбираются режимы; «Минимальная стоимость» — трасса с наименьшим показателем S. Почти тот же коридор не отдаётся.
- Официальное техническое приложение в `config/appendix.yml`.
- Конкурсный набор `!!!_Датасет.geojson` разбирается и считается движком леса (`engine.flow`: граф видимости, DN по расходу и предельной длине, камеры, врезки только в существующие камеры, рейтинг 70/30). Реконструкция существующей сети по актуальному приложению не считается.
- Выгрузка сдачи: `GET /api/v1/jobs/{id}/result.geojson` и `samples/contest-result.geojson`.

## Финальная

- [x] Расчёт конкурсного набора движком `engine.flow` по приложению от 26.09.2026: 17/17 ОКС, «Минимальная стоимость» **248,5 млн ₽** (1784 м, S = 12,312), «Минимум врезок» **252,1 млн ₽** (1832 м). Файл `samples/contest-result.geojson`
- [x] Выгрузка `samples/contest-result.geojson` (`make contest` или `java -jar target/heatnet.jar --process-contest`)
- [x] Врезка в существующую камеру входит в `variant_summary` (`existing_chamber_tie_in_cost`, 5 млн ₽ за участок). Отдельного объекта `tie_in` и реконструкции в файле нет
- [ ] `docker-compose up --build` на Ubuntu 22
- [ ] Презентация
- [x] Сопроводиловка: ARCHITECTURE, ALGORITHM, DATA-CONTRACT, SCREEN, описание API `/api.html`
