package com.warehouse_delivery.spring_boot.entity;

import jakarta.persistence.Entity;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;

@Entity
public class DroneModel {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    private String manufacturer;

    private double maxSpeed;

    private double recommendedSpeed;

    private double batteryCapacity;

    private double maxTripLength;

    private double maxPayloadCapacity;

    // Getters and setters

    public Long getId() {
        return id;
    }

    public void setId(Long id) {
        this.id = id;
    }

    public String getManufacturer() {
        return manufacturer;
    }

    public void setManufacturer(String manufacturer) {
        this.manufacturer = manufacturer;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public double getRecommendedSpeed() {
        return recommendedSpeed;
    }

    public void setRecommendedSpeed(double recommendedSpeed) {
        this.recommendedSpeed = recommendedSpeed;
    }

    public double getBatteryCapacity() {
        return batteryCapacity;
    }

    public void setBatteryCapacity(double batteryCapacity) {
        this.batteryCapacity = batteryCapacity;
    }

    public double getMaxTripLength() {
        return maxTripLength;
    }

    public void setMaxTripLength(double maxTripLength) {
        this.maxTripLength = maxTripLength;
    }

    public double getMaxPayloadCapacity() {
        return maxPayloadCapacity;
    }

    public void setMaxPayloadCapacity(double maxPayloadCapacity) {
        this.maxPayloadCapacity = maxPayloadCapacity;
    }

}