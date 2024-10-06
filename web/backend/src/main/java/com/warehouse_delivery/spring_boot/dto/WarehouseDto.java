package com.warehouse_delivery.spring_boot.dto;

import com.warehouse_delivery.spring_boot.entity.Address;

public class WarehouseDto {

    private Long id;
    private String name;
    private double latitude;
    private double longitude;
    private Address address;
    private int capacity;

    // Constructors, getters, and setters

    public WarehouseDto() {}

    public WarehouseDto(Long id, String name, double latitude, double longitude, Address address, int capacity) {
        this.id = id;
        this.name = name;
        this.latitude = latitude;
        this.longitude = longitude;
        this.address = address;
        this.capacity = capacity;
    }

    public Long getId() {
        return id;
    }

    public void setId(Long id) {
        this.id = id;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public double getLatitude() {
        return latitude;
    }

    public void setLatitude(double latitude) {
        this.latitude = latitude;
    }

    public double getLongitude() {
        return longitude;
    }

    public void setLongitude(double longitude) {
        this.longitude = longitude;
    }

    public Address getAddress() {
        return address;
    }

    public void setAddress(Address address) {
        this.address = address;
    }

    public int getCapacity() {
        return capacity;
    }

    public void setCapacity(int capacity) {
        this.capacity = capacity;
    }
}