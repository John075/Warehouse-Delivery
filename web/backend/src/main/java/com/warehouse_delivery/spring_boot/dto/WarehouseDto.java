package com.warehouse_delivery.spring_boot.dto;

import com.warehouse_delivery.spring_boot.entity.Address;

public class WarehouseDto {

    private Long id;
    private String name;
    private Address address;
    private int capacity;

    // Constructors, getters, and setters

    public WarehouseDto() {}

    public WarehouseDto(Long id, String name, Address address, int capacity) {
        this.id = id;
        this.name = name;
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