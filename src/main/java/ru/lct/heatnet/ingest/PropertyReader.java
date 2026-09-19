package ru.lct.heatnet.ingest;

import com.fasterxml.jackson.databind.JsonNode;
import java.util.Locale;

public final class PropertyReader {

    private final JsonNode props;

    public PropertyReader(JsonNode props) {
        this.props = props;
    }

    public String str(String... keys) {
        if (props == null || props.isNull()) {
            return null;
        }
        for (String key : keys) {
            JsonNode n = props.get(key);
            if (n != null && !n.isNull() && !n.asText("").isBlank()) {
                return n.asText();
            }
        }
        return null;
    }

    public Double num(String... keys) {
        if (props == null || props.isNull()) {
            return null;
        }
        for (String key : keys) {
            JsonNode n = props.get(key);
            if (n == null || n.isNull()) {
                continue;
            }
            if (n.isNumber()) {
                return n.doubleValue();
            }
            try {
                String t = n.asText("").replace(",", ".");
                if (!t.isBlank()) {
                    return Double.parseDouble(t);
                }
            } catch (NumberFormatException ignored) {
                // next alias
            }
        }
        return null;
    }

    public String firstNonBlank(Iterable<String> keys) {
        for (String key : keys) {
            String v = str(key);
            if (v != null) {
                return v;
            }
        }
        return null;
    }

    public Double firstNum(Iterable<String> keys) {
        for (String key : keys) {
            Double v = num(key);
            if (v != null) {
                return v;
            }
        }
        return null;
    }

    public static String norm(String value) {
        if (value == null) {
            return "";
        }
        return value.trim().toLowerCase(Locale.ROOT).replace('ё', 'е');
    }
}
