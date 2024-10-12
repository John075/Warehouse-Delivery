package com.warehouse_delivery.spring_boot.entity;

import jakarta.persistence.Embeddable;

@Embeddable
public class Address {

    private double latitude;
    private double longitude;

    // Constructors, getters, and setters

    public Address() {}

    public Address(double latitude, double longitude) {
        this.latitude = latitude;
        this.longitude = longitude;
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
}
