package ru.lct.heatnet.service;

import org.springframework.boot.ApplicationArguments;
import org.springframework.boot.ApplicationRunner;
import org.springframework.stereotype.Component;

@Component
public class StorageBootstrap implements ApplicationRunner {

    private final FileStorageService storage;

    public StorageBootstrap(FileStorageService storage) {
        this.storage = storage;
    }

    @Override
    public void run(ApplicationArguments args) {
        storage.root();
    }
}
