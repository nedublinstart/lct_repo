package ru.lct.heatnet.api.dto;

public class ErrorResponse {
    public String error;
    public String details;

    public static ErrorResponse of(String error, String details) {
        ErrorResponse r = new ErrorResponse();
        r.error = error;
        r.details = details;
        return r;
    }
}
