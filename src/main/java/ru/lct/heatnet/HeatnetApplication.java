package ru.lct.heatnet;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.boot.context.properties.EnableConfigurationProperties;
import org.springframework.scheduling.annotation.EnableAsync;
import ru.lct.heatnet.config.HeatnetProperties;

@SpringBootApplication
@EnableAsync
@EnableConfigurationProperties(HeatnetProperties.class)
public class HeatnetApplication {

    public static void main(String[] args) {
        SpringApplication.run(HeatnetApplication.class, args);
    }
}
