package com.warehouse_delivery.spring_boot.utils;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import java.time.LocalDateTime;

public class TestUtils {

    public static String asJsonString(final Object obj) {
        try {
            Gson gson = new GsonBuilder()
                    .registerTypeAdapter(LocalDateTime.class, new LocalDateTimeAdapter())  // Register adapter
                    .create();

            return gson.toJson(obj);  // Use Gson to serialize object to JSON
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}