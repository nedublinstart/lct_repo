package ru.lct.heatnet.config;

import io.swagger.v3.oas.models.OpenAPI;
import io.swagger.v3.oas.models.info.Info;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

@Configuration
public class OpenApiConfig {

    @Bean
    public OpenAPI heatnetOpenApi() {
        return new OpenAPI().info(new Info()
                .title("Теплотрасса")
                .version("0.1.0")
                .description("Загрузка GeoJSON, расчёт трасс подключения ОКС и выгрузка результата. "
                        + "Режимы PLAN_2D и DEPTH. Карта: `/`. Описание полей и методов: `/api.html`."));
    }
}
