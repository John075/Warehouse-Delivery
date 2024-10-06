package com.warehouse_delivery.spring_boot.services;

import com.warehouse_delivery.spring_boot.dto.DroneModelDto;

import java.util.List;

public interface DroneModelService {

    List<DroneModelDto> getAllDroneModels();
    DroneModelDto getDroneModelById(Long id);
    DroneModelDto createDroneModel(DroneModelDto droneModel);
    DroneModelDto updateDroneModel(Long id, DroneModelDto updatedDroneModel);
    void deleteDroneModel(Long id);
}
