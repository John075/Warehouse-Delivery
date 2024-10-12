package com.warehouse_delivery.spring_boot.dto;

import com.fasterxml.jackson.annotation.JsonInclude;
import com.warehouse_delivery.spring_boot.entity.Address;
import com.warehouse_delivery.spring_boot.enums.PackageStatus;

public class PackageDto {

    private Long id;
    private String name;
    private PackageStatus status;
    private Address destination;

    @JsonInclude(JsonInclude.Include.NON_NULL)
    private DroneDto assignedDrone;
    private WarehouseDto warehouse;
    private int priority;
    private long orderTime;

    // Constructors, getters, and setters

    public PackageDto() {}

    public PackageDto(Long id, String name, PackageStatus status, Address destination, DroneDto assignedDrone, WarehouseDto warehouse, int priority, long orderTime) {
        this.id = id;
        this.name = name;
        this.status = status;
        this.destination = destination;
        this.assignedDrone = assignedDrone;
        this.warehouse = warehouse;
        this.priority = priority;
        this.orderTime = orderTime;
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

    public PackageStatus getStatus() {
        return status;
    }

    public void setStatus(PackageStatus status) {
        this.status = status;
    }

    public Address getDestination() {
        return destination;
    }

    public void setDestination(Address destination) {
        this.destination = destination;
    }

    public DroneDto getAssignedDrone() {
        return assignedDrone;
    }

    public void setAssignedDrone(DroneDto assignedDrone) {
        this.assignedDrone = assignedDrone;
    }

    public WarehouseDto getWarehouse() {
        return warehouse;
    }

    public void setWarehouse(WarehouseDto warehouse) {
        this.warehouse = warehouse;
    }

    public int getPriority() {
        return priority;
    }

    public void setPriority(int priority) {
        this.priority = priority;
    }

    public long getOrderTime() {
        return orderTime;
    }

    public void setOrderTime(long orderTime) {
        this.orderTime = orderTime;
    }
}