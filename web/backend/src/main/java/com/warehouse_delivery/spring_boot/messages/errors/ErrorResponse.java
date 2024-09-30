package com.warehouse_delivery.spring_boot.messages.errors;

public class ErrorResponse {

    private String error;

    public ErrorResponse (final String error) {
        this.error = error;
    }

    public String getError () {
        return error;
    }
    public void setError (final String error) {
        this.error = error;
    }
}
