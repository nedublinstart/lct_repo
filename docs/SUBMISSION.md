# Сдача

Поля формы. В поле репозитория указывается открытый адрес ниже. Расчёт, инструкция и аннотация должны лежать в его `main`.

Репозиторий

https://github.com/nedublinstart/lct_repo

В корне `README.md`: подъём через `docker-compose up --build -d` на Ubuntu Server 22.04. Офлайн-прогон конкурсного файла: `make contest`.

Документация

https://github.com/nedublinstart/lct_repo/blob/main/docs/ARCHITECTURE.md

Дальше по тому же дереву: `docs/ALGORITHM.md`, `docs/MODEL.md`, `docs/ANNOTATION.md`, `docs/DATA-CONTRACT.md`. Описание методов после запуска: http://localhost:8080/api.html

Прототип

Публичного стенда нет. После `docker-compose up --build -d` из README:

- http://localhost:8080 — карта, загрузка, расчёт
- http://localhost:8080/api.html — поля и методы
- http://localhost:8080/swagger-ui.html — Swagger

Готовый результат конкурсного набора: `samples/contest-result.geojson`. «Минимальная стоимость» — 248 546 610 ₽, 1784 м, S = 12,312, 17/17 ОКС. «Минимум врезок» — 252 093 904 ₽, 1832 м, S = 12,554. «Минимальная длина» — 271 679 549 ₽, 1713 м, S = 12,747.

Дополнительные материалы

- https://github.com/nedublinstart/lct_repo/blob/main/samples/contest-result.geojson
- https://github.com/nedublinstart/lct_repo/blob/main/docs/MODEL.md
- https://github.com/nedublinstart/lct_repo/blob/main/docs/ANNOTATION.md
- https://github.com/nedublinstart/lct_repo/blob/main/docs/DEMO.md
- https://github.com/nedublinstart/lct_repo/blob/main/docs/SCREEN.md
- https://github.com/nedublinstart/lct_repo/blob/main/config/appendix.yml
