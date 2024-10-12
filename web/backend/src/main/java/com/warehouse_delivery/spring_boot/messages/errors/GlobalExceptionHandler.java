package com.warehouse_delivery.spring_boot.messages.errors;

import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.ControllerAdvice;
import org.springframework.web.bind.annotation.ExceptionHandler;

@ControllerAdvice
public class GlobalExceptionHandler {

    @ExceptionHandler ( GeneralErrorException.class )
    public ResponseEntity<ErrorResponse> handleCustomBadRequestException ( final GeneralErrorException ex ) {
        return ResponseEntity.status( HttpStatus.BAD_REQUEST ).body( new ErrorResponse( ex.getMessage() ) );
    }

    @ExceptionHandler ( ResourceNotFoundException.class )
    public ResponseEntity<ErrorResponse> handleCustomBadRequestException ( final ResourceNotFoundException ex ) {
        return ResponseEntity.status( HttpStatus.NOT_FOUND ).body( new ErrorResponse( ex.getMessage() ) );
    }

    //Fallback handler
    @ExceptionHandler(Exception.class)
    public ResponseEntity<String> handleAllExceptions(Exception ex) {
        ex.printStackTrace();
        return new ResponseEntity<>("Error occurred: " + ex.getMessage(), HttpStatus.INTERNAL_SERVER_ERROR);
    }
}
