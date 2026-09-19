package ru.lct.heatnet.api;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;
import org.springframework.http.MediaType;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;
import ru.lct.heatnet.appendix.AppendixLoader;
import ru.lct.heatnet.appendix.AppendixModel;
import ru.lct.heatnet.config.HeatnetProperties;

@RestController
@RequestMapping("/api/v1/appendix")
public class AppendixController {

    private final AppendixLoader loader;
    private final HeatnetProperties properties;

    public AppendixController(AppendixLoader loader, HeatnetProperties properties) {
        this.loader = loader;
        this.properties = properties;
    }

    @GetMapping
    public AppendixModel parsed() {
        return loader.load();
    }

    @GetMapping(value = "/raw", produces = MediaType.TEXT_PLAIN_VALUE)
    public String raw() throws Exception {
        Path path = Path.of(properties.getAppendixPath());
        if (Files.exists(path)) {
            return Files.readString(path);
        }
        return "используется classpath:/appendix/default.yml";
    }

    @GetMapping("/meta")
    public Map<String, Object> meta() {
        return Map.of(
                "appendixPath", properties.getAppendixPath(),
                "storageDir", properties.getStorageDir(),
                "maxUploadBytes", properties.getMaxUploadBytes(),
                "async", properties.isAsync()
        );
    }
}
