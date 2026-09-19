package ru.lct.heatnet.ingest;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonToken;
import java.io.IOException;
import java.io.StringWriter;
import org.locationtech.jts.geom.Envelope;

/**
 * Копирует один geometry-объект из Jackson-парсера в строку, не собирая JsonNode,
 * и попутно считает bbox по координатам.
 */
final class GeometryTokenCopier {

    private GeometryTokenCopier() {
    }

    static Copied copy(JsonParser parser, JsonFactory factory) throws IOException {
        Copied copied = new Copied();
        if (parser.currentToken() == JsonToken.VALUE_NULL || parser.currentToken() == null) {
            copied.json = "null";
            return copied;
        }
        StringWriter out = new StringWriter(256);
        try (JsonGenerator generator = factory.createGenerator(out)) {
            copyValue(parser, generator, copied);
        }
        copied.json = out.toString();
        return copied;
    }

    private static void copyValue(JsonParser parser, JsonGenerator generator, Copied copied) throws IOException {
        JsonToken token = parser.currentToken();
        if (token == null) {
            return;
        }
        switch (token) {
            case START_OBJECT:
                generator.writeStartObject();
                while (parser.nextToken() != JsonToken.END_OBJECT) {
                    String name = parser.currentName();
                    generator.writeFieldName(name);
                    parser.nextToken();
                    if ("type".equals(name) && parser.currentToken() == JsonToken.VALUE_STRING) {
                        copied.type = parser.getText();
                        generator.writeString(copied.type);
                    } else if ("coordinates".equals(name)) {
                        copied.inCoordinates = true;
                        copyValue(parser, generator, copied);
                        copied.inCoordinates = false;
                    } else {
                        copyValue(parser, generator, copied);
                    }
                }
                generator.writeEndObject();
                return;
            case START_ARRAY:
                generator.writeStartArray();
                int numbers = 0;
                double x = 0;
                while (parser.nextToken() != JsonToken.END_ARRAY) {
                    JsonToken item = parser.currentToken();
                    if (copied.inCoordinates && item != null && item.isNumeric()) {
                        double v = parser.getDoubleValue();
                        generator.writeNumber(v);
                        numbers++;
                        if (numbers == 1) {
                            x = v;
                        } else if (numbers == 2) {
                            copied.envelope.expandToInclude(x, v);
                        }
                    } else {
                        copyValue(parser, generator, copied);
                    }
                }
                generator.writeEndArray();
                return;
            case VALUE_STRING:
                generator.writeString(parser.getText());
                return;
            case VALUE_NUMBER_INT:
                generator.writeNumber(parser.getLongValue());
                return;
            case VALUE_NUMBER_FLOAT:
                generator.writeNumber(parser.getDoubleValue());
                return;
            case VALUE_TRUE:
                generator.writeBoolean(true);
                return;
            case VALUE_FALSE:
                generator.writeBoolean(false);
                return;
            case VALUE_NULL:
                generator.writeNull();
                return;
            default:
                generator.copyCurrentEvent(parser);
        }
    }

    static final class Copied {
        String json;
        String type;
        final Envelope envelope = new Envelope();
        boolean inCoordinates;
    }
}
