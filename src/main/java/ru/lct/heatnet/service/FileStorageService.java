package ru.lct.heatnet.service;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.UUID;
import org.springframework.stereotype.Service;
import org.springframework.web.multipart.MultipartFile;
import ru.lct.heatnet.config.HeatnetProperties;

@Service
public class FileStorageService {

    private final HeatnetProperties properties;

    public FileStorageService(HeatnetProperties properties) {
        this.properties = properties;
    }

    public Path root() {
        Path dir = Path.of(properties.getStorageDir()).toAbsolutePath().normalize();
        try {
            Files.createDirectories(dir.resolve("uploads"));
            Files.createDirectories(dir.resolve("results"));
            Files.createDirectories(dir.resolve("tmp"));
        } catch (IOException e) {
            throw new IllegalStateException("Не удалось создать каталоги хранения", e);
        }
        return dir;
    }

    public Path saveUpload(UUID datasetId, MultipartFile file) {
        try {
            Path dest = root().resolve("uploads").resolve(datasetId + ".geojson");
            file.transferTo(dest);
            return dest;
        } catch (IOException e) {
            throw new IllegalStateException("Не удалось сохранить загрузку на диск", e);
        }
    }

    public Path saveUpload(UUID datasetId, InputStream in, String suffix) {
        try {
            Path dest = root().resolve("uploads").resolve(datasetId + suffix);
            Files.copy(in, dest, java.nio.file.StandardCopyOption.REPLACE_EXISTING);
            return dest;
        } catch (IOException e) {
            throw new IllegalStateException("Не удалось сохранить поток на диск", e);
        }
    }

    public Path writeResult(UUID jobId, int rank, String geoJson) {
        try {
            Path dest = root().resolve("results").resolve(jobId + "-v" + rank + ".geojson");
            Files.writeString(dest, geoJson, StandardCharsets.UTF_8);
            return dest;
        } catch (IOException e) {
            throw new IllegalStateException("Не удалось записать результат", e);
        }
    }

    public String readResult(Path path) {
        try {
            return Files.readString(path, StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new IllegalStateException("Не удалось прочитать результат", e);
        }
    }
}
