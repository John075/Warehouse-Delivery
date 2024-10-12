package com.warehouse_delivery.spring_boot.services.impl;

import com.warehouse_delivery.spring_boot.dto.DroneModelDto;
import com.warehouse_delivery.spring_boot.entity.DroneModel;
import com.warehouse_delivery.spring_boot.mapper.DroneModelMapper;
import com.warehouse_delivery.spring_boot.messages.errors.ResourceNotFoundException;
import com.warehouse_delivery.spring_boot.repositories.DroneModelRepository;
import com.warehouse_delivery.spring_boot.services.DroneModelService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.Comparator;
import java.util.List;

@Service
public class DroneModelServiceImpl implements DroneModelService {

    @Autowired
    private DroneModelRepository droneModelRepository;

    /**
     * Retrieve all drone models from the repository.
     *
     * @return List of DroneModelDto.
     */
    @Override
    public List<DroneModelDto> getAllDroneModels() {
        List<DroneModel> droneModels = droneModelRepository.findAll();
        droneModels.sort(Comparator.comparing(DroneModel::getId));
        return droneModels.stream()
                .map(DroneModelMapper::mapToDroneModelDto)
                .toList();
    }

    /**
     * Retrieve a drone model by its ID.
     *
     * @param id The ID of the drone model.
     * @return The DroneModelDto or null if not found.
     */
    @Override
    public DroneModelDto getDroneModelById(Long id) {
        DroneModel droneModel = droneModelRepository.findById(id).orElseThrow(() -> new ResourceNotFoundException("Drone not found with ID " + id));
        return DroneModelMapper.mapToDroneModelDto(droneModel);
    }

    /**
     * Create a new drone model.
     *
     * @param droneModelDto The DroneModelDto to be saved.
     * @return The saved DroneModelDto.
     */
    @Override
    public DroneModelDto createDroneModel(DroneModelDto droneModelDto) {
        DroneModel droneModel = DroneModelMapper.mapToDroneModel(droneModelDto);
        DroneModel savedDroneModel = droneModelRepository.save(droneModel);
        return DroneModelMapper.mapToDroneModelDto(savedDroneModel);
    }

    /**
     * Update an existing drone model by its ID.
     *
     * @param id The ID of the drone model to update.
     * @param updatedDroneModelDto The new drone model data.
     * @return The updated DroneModelDto or null if not found.
     */
    @Override
    public DroneModelDto updateDroneModel(Long id, DroneModelDto updatedDroneModelDto) {
        droneModelRepository.findById(id).orElseThrow(() -> new ResourceNotFoundException("Drone not found with ID " + id));
        DroneModel updatedModel = droneModelRepository.save(DroneModelMapper.mapToDroneModel(updatedDroneModelDto));
        return DroneModelMapper.mapToDroneModelDto(updatedModel);
    }

    /**
     * Delete a drone model by its ID.
     *
     * @param id The ID of the drone model to delete.
     */
    @Override
    public void deleteDroneModel(Long id) {
        DroneModel droneModel = droneModelRepository.findById(id).orElseThrow(() -> new ResourceNotFoundException("Drone not found with ID " + id));
        droneModelRepository.delete(droneModel);
    }
}