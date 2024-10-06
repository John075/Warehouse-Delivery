package com.warehouse_delivery.spring_boot.messages;

public class GeneralResponse {

    private String message;

    public GeneralResponse(final String message) {
        this.message = message;
    }

    public String getMessage() {return this.message; }
    public void setMessage (final String message) {
        this.message = message;
    }
}
