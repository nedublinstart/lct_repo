package ru.lct.heatnet.appendix;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.stereotype.Component;
import ru.lct.heatnet.config.HeatnetProperties;

@Component
public class AppendixLoader {

    private static final Logger log = LoggerFactory.getLogger(AppendixLoader.class);
    private final HeatnetProperties properties;
    private final ObjectMapper yaml = new ObjectMapper(new YAMLFactory());

    public AppendixLoader(HeatnetProperties properties) {
        this.properties = properties;
    }

    public AppendixModel load() {
        Path path = Path.of(properties.getAppendixPath());
        try {
            if (Files.exists(path)) {
                log.info("Читаю техническое приложение: {}", path.toAbsolutePath());
                return yaml.readValue(path.toFile(), AppendixModel.class);
            }
            try (InputStream in = getClass().getResourceAsStream("/appendix/default.yml")) {
                if (in == null) {
                    throw new IllegalStateException("Не найден ни файл приложения, ни classpath default.yml");
                }
                log.warn("Файл {} отсутствует, использую встроенный default.yml", path);
                return yaml.readValue(in, AppendixModel.class);
            }
        } catch (IOException e) {
            throw new IllegalStateException("Не удалось прочитать техническое приложение: " + e.getMessage(), e);
        }
    }
}
