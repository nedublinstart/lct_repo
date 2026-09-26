package ru.lct.heatnet.api;

/**
 * Текст, который уходит в интерфейс. Английское сообщение исключения сюда не попадает.
 */
public final class UserFacing {

    private UserFacing() {
    }

    public static String cyrillicOr(String text, String fallback) {
        if (text == null || text.isBlank()) {
            return fallback;
        }
        int i = 0;
        while (i < text.length()) {
            int cp = text.codePointAt(i);
            if (Character.UnicodeScript.of(cp) == Character.UnicodeScript.CYRILLIC) {
                return text.length() > 500 ? text.substring(0, 500) : text;
            }
            i += Character.charCount(cp);
        }
        return fallback;
    }
}
