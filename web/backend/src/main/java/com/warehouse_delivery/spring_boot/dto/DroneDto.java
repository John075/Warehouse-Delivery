package com.warehouse_delivery.spring_boot.dto;

public class DroneDto {

    /** Fields **/
    private String name;
    private Long id;

    public DroneDto() {}

    /** Setters **/
    public void setName(String name) {
        this.name = name;
    }

    public void setId(final Long id) {
        this.id = id;
    }

    /** Getters **/
    public String getName() {
        return this.name;
    }

    public Long getId() {
        return this.id;
    }

}
