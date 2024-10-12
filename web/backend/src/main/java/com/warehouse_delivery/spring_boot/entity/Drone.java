package com.warehouse_delivery.spring_boot.entity;

import com.warehouse_delivery.spring_boot.broadcaster.DroneEntityListener;
import com.warehouse_delivery.spring_boot.enums.DroneStatus;
import jakarta.persistence.*;

import java.time.LocalDateTime;
import java.util.List;
import java.util.ArrayList;

@Entity
@EntityListeners(DroneEntityListener.class)
public class Drone {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    private String name;

    @OneToMany(mappedBy = "assignedDrone", cascade = CascadeType.ALL, fetch = FetchType.LAZY)
    private List<Package> packages = new ArrayList<>();

    @Enumerated(EnumType.STRING)
    private DroneStatus status;

    @ManyToOne
    private DroneModel model;

    @ManyToOne
    private Warehouse warehouse;

    @ElementCollection
    private List<String> alerts = new ArrayList<>();

    private double lastLatitudeLocation;
    private double lastLongitudeLocation;

    private double batteryLife;

    private LocalDateTime lastMaintenanceDate;

    private boolean connectedToSystem;

    // Getters and Setters

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

    public List<Package> getPackages() {
        return packages;
    }

    public void setPackages(List<Package> packages) {
        this.packages = packages;
    }

    public DroneStatus getStatus() {
        return status;
    }

    public void setStatus(DroneStatus status) {
        this.status = status;
    }

    public Warehouse getWarehouse() {
        return warehouse;
    }

    public void setWarehouse(Warehouse warehouse) {
        this.warehouse = warehouse;
    }

    public List<String> getAlerts() {
        return alerts;
    }

    public void setAlerts(List<String> alerts) {
        this.alerts = alerts;
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

    public LocalDateTime getLastMaintenanceDate() {
        return lastMaintenanceDate;
    }

    public void setLastMaintenanceDate(LocalDateTime lastMaintenanceDate) {
        this.lastMaintenanceDate = lastMaintenanceDate;
    }

    public boolean isConnectedToSystem() {
        return connectedToSystem;
    }

    public void setConnectedToSystem(boolean connectedToSystem) {
        this.connectedToSystem = connectedToSystem;
    }

    public DroneModel getDroneModel() {
        return model;
    }

    public void setDroneModel(DroneModel model) {
        this.model = model;
    }
}