package ru.lct.heatnet.api;

import java.util.NoSuchElementException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.ExceptionHandler;
import org.springframework.web.bind.annotation.RestControllerAdvice;
import org.springframework.web.server.ResponseStatusException;
import ru.lct.heatnet.api.dto.ErrorResponse;

@RestControllerAdvice
public class ApiExceptionHandler {

    private static final Logger log = LoggerFactory.getLogger(ApiExceptionHandler.class);

    @ExceptionHandler(ResponseStatusException.class)
    public ResponseEntity<ErrorResponse> status(ResponseStatusException e) {
        HttpStatus status = HttpStatus.resolve(e.getStatus().value());
        String title = httpRu(status);
        return ResponseEntity.status(e.getStatus()).body(ErrorResponse.of(
                title,
                UserFacing.cyrillicOr(e.getReason(), title)));
    }

    @ExceptionHandler(NoSuchElementException.class)
    public ResponseEntity<ErrorResponse> missing(NoSuchElementException e) {
        return ResponseEntity.status(HttpStatus.NOT_FOUND)
                .body(ErrorResponse.of("Не найдено", UserFacing.cyrillicOr(e.getMessage(), "Не найдено")));
    }

    @ExceptionHandler(Exception.class)
    public ResponseEntity<ErrorResponse> other(Exception e) {
        log.error("Unhandled", e);
        return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR)
                .body(ErrorResponse.of("Ошибка сервера", "Внутренняя ошибка сервера"));
    }

    private static String httpRu(HttpStatus status) {
        if (status == null) {
            return "Ошибка";
        }
        switch (status) {
            case BAD_REQUEST:
                return "Некорректный запрос";
            case CONFLICT:
                return "Конфликт состояния";
            case NOT_FOUND:
                return "Не найдено";
            case PAYLOAD_TOO_LARGE:
                return "Файл больше 3 ГБ";
            case INTERNAL_SERVER_ERROR:
                return "Ошибка сервера";
            default:
                return "Ошибка запроса";
        }
    }
}
