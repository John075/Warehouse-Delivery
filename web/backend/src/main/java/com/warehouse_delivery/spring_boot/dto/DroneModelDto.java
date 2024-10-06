package com.warehouse_delivery.spring_boot.dto;

public class DroneModelDto {

    private Long id;
    private String manufacturer;
    private double maxSpeed;
    private double recommendedSpeed;
    private double batteryCapacity;
    private double maxTripLength;

    // Constructors, getters, and setters

    public DroneModelDto() {}

    public DroneModelDto(Long id, String manufacturer, double maxSpeed, double recommendedSpeed, double batteryCapacity, double maxTripLength) {
        this.id = id;
        this.manufacturer = manufacturer;
        this.maxSpeed = maxSpeed;
        this.recommendedSpeed = recommendedSpeed;
        this.batteryCapacity = batteryCapacity;
        this.maxTripLength = maxTripLength;
    }

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
}