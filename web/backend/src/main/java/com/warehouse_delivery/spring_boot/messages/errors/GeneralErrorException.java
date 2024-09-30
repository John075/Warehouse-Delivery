package com.warehouse_delivery.spring_boot.messages.errors;

import org.springframework.http.HttpStatus;
import org.springframework.web.bind.annotation.ResponseStatus;

@ResponseStatus ( value = HttpStatus.BAD_REQUEST )
public class GeneralErrorException extends RuntimeException {

    private static final long serialVersionUID = 1L;

    public GeneralErrorException (final String message) {
        super( message );
    }
}
