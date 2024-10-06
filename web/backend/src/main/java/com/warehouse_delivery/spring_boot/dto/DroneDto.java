package com.warehouse_delivery.spring_boot.dto;

import com.warehouse_delivery.spring_boot.enums.DroneStatus;

import java.time.LocalDateTime;
import java.util.List;

public class DroneDto {

    private Long id;
    private String name;
    private DroneStatus status;
    private double lastLatitudeLocation;
    private double lastLongitudeLocation;
    private double batteryLife;
    private boolean connectedToSystem;
    private WarehouseDto warehouse;
    private LocalDateTime lastMaintenanceDate;
    private List<PackageDto> packages;

    // Constructors, getters, and setters

    public DroneDto() {}

    public DroneDto(Long id, String name, DroneStatus status, double lastLatitudeLocation, double lastLongitudeLocation, double batteryLife, WarehouseDto warehouse, LocalDateTime lastMaintenanceDate, List<PackageDto> packages, boolean connectedToSystem) {
        this.id = id;
        this.name = name;
        this.status = status;
        this.lastLatitudeLocation = lastLatitudeLocation;
        this.lastLongitudeLocation = lastLongitudeLocation;
        this.batteryLife = batteryLife;
        this.warehouse = warehouse;
        this.lastMaintenanceDate = lastMaintenanceDate;
        this.packages = packages;
        this.connectedToSystem = connectedToSystem;
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

    public DroneStatus getStatus() {
        return status;
    }

    public void setStatus(DroneStatus status) {
        this.status = status;
    }

    public double getLastLatitudeLocation() {
        return lastLatitudeLocation;
    }

    public void setLastLatitudeLocation(double lastLatitudeLocation) {
        this.lastLatitudeLocation = lastLatitudeLocation;
    }

    public double getLastLongitudeLocation() {
        return lastLongitudeLocation;
    }

    public void setLastLongitudeLocation(double lastLongitudeLocation) {
        this.lastLongitudeLocation = lastLongitudeLocation;
    }

    public double getBatteryLife() {
        return batteryLife;
    }

    public void setBatteryLife(double batteryLife) {
        this.batteryLife = batteryLife;
    }

    public WarehouseDto getWarehouse() {
        return warehouse;
    }

    public void setWarehouse(WarehouseDto warehouse) {
        this.warehouse = warehouse;
    }

    public LocalDateTime getLastMaintenanceDate() {
        return lastMaintenanceDate;
    }

    public void setLastMaintenanceDate(LocalDateTime lastMaintenanceDate) {
        this.lastMaintenanceDate = lastMaintenanceDate;
    }

    public List<PackageDto> getPackages() {
        return packages;
    }

    public void setPackages(List<PackageDto> packages) {
        this.packages = packages;
    }

    public boolean isConnectedToSystem() {
        return connectedToSystem;
    }

    public void setConnectedToSystem(boolean connectedToSystem) {
        this.connectedToSystem = connectedToSystem;
    }
}