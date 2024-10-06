package com.warehouse_delivery.spring_boot.services;

import com.warehouse_delivery.spring_boot.dto.DroneDto;

import java.util.List;

public interface DroneService {
    DroneDto getDrone(Long id);
    List<DroneDto> getAllDrones();
    DroneDto registerDrone(DroneDto droneDto);
    DroneDto updateDrone(Long id, DroneDto droneDto);
    void deleteDrone(Long id);
}