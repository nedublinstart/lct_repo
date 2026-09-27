# Сдача

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
