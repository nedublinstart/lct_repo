package ru.lct.heatnet.config;

import io.swagger.v3.oas.models.OpenAPI;
import io.swagger.v3.oas.models.info.Info;
import io.swagger.v3.oas.models.info.License;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

@Configuration
public class OpenApiConfig {

    @Bean
    public OpenAPI heatnetOpenApi() {
        return new OpenAPI().info(new Info()
                .title("ТеплоТрасса API")
                .version("0.1.0")
                .description("Загрузка совмещённого GeoJSON, расчёт вариантов подключения ОКС "
                        + "к тепловой сети и выгрузка результата. "
                        + "UI: `/`, Swagger: `/swagger-ui.html`.")
                .license(new License().name("Internal hackathon")));
    }
}
