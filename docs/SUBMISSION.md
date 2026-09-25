# Сдача

## Промежуточная / текущее состояние

- Каркас Java 11 + Spring Boot 2.6.3 + springdoc 1.7.0 + docker-compose 2.4.
- Потоковая загрузка GeoJSON, PostgreSQL/H2, карта, три автоварианта.
- Официальное техническое приложение в `config/appendix.yml`.
- Конкурсный набор `!!!_Датасет.geojson` разбирается и считается Smart-движком (дерево камер, выход ИТП из зданий, реконструкция, рейтинг 70/30).
- Выгрузка сдачи: `GET /api/v1/jobs/{id}/result.geojson` и `samples/contest-result.geojson`.

## Финальная

- [x] Расчёт конкурсного набора: `StreetFrameTest` — 17/17, **188.8 млн ₽**, 2 врезки, 0 новых камер, реконструкции нет
- [x] Выгрузка `samples/contest-result.geojson` (`make contest` или `java -jar target/heatnet.jar --process-contest`)
- [x] Врезка в выгрузке только если к ней приходит труба и точка лежит на существующей сети
- [ ] `docker-compose up --build` на Ubuntu 22
- [ ] Презентация
- [ ] Сопроводиловка: ARCHITECTURE, ALGORITHM, DATA-CONTRACT, границы
