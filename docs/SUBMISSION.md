# Сдача

## Промежуточная / текущее состояние

- Каркас Java 11 + Spring Boot 2.6.3 + springdoc 1.7.0 + docker-compose 2.4.
- Потоковая загрузка GeoJSON, PostgreSQL/H2, карта, три автоварианта.
- Официальное техническое приложение в `config/appendix.yml`.
- Конкурсный набор `!!!_Датасет.geojson` разбирается и считается Smart-движком (дерево камер, выход ИТП из зданий, реконструкция, рейтинг 70/30).
- Выгрузка сдачи: `GET /api/v1/jobs/{id}/result.geojson` и `samples/contest-result.geojson`.

## Финальная

- [ ] `docker-compose up --build` на Ubuntu 22
- [ ] Выгрузка GeoJSON конкурсного набора (`make contest` или кнопка «Конкурсный набор»)
- [ ] Презентация
- [ ] Сопроводиловка: ARCHITECTURE, ALGORITHM, DATA-CONTRACT, границы
