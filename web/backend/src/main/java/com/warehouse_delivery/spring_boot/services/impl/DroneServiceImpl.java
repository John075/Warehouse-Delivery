package com.warehouse_delivery.spring_boot.services.impl;

import com.warehouse_delivery.spring_boot.dto.DroneDto;
import com.warehouse_delivery.spring_boot.entity.Drone;
import com.warehouse_delivery.spring_boot.mapper.DroneMapper;
import com.warehouse_delivery.spring_boot.messages.errors.ResourceNotFoundException;
import com.warehouse_delivery.spring_boot.repositories.DroneRepository;
import com.warehouse_delivery.spring_boot.services.DroneService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
public class DroneServiceImpl implements DroneService {

    private final DroneRepository droneRepository;

    @Autowired
    public DroneServiceImpl(DroneRepository droneRepository) {
        this.droneRepository = droneRepository;
    }

    @Override
    public DroneDto getDrone(Long id) {
        Drone drone = droneRepository.findById(id)
                .orElseThrow(() -> new ResourceNotFoundException("Drone not found with id: " + id));
        return DroneMapper.mapToDroneDto(drone);
    }

    @Override
    public List<DroneDto> getAllDrones() {
        List<Drone> drones = droneRepository.findAll();
        return drones.stream().map(DroneMapper::mapToDroneDto).toList();
    }

    @Override
    public DroneDto registerDrone(DroneDto droneDto) {
        Drone newDrone = DroneMapper.mapToDrone(droneDto);
        droneRepository.save(newDrone);
        return DroneMapper.mapToDroneDto(newDrone);
    }

    @Override
    public DroneDto updateDrone(Long id, DroneDto droneDto) {
        droneRepository.findById(id)
                .orElseThrow(() -> new ResourceNotFoundException("Drone not found with id: " + id));

        Drone updatedDrone = droneRepository.save(DroneMapper.mapToDrone(droneDto));
        return DroneMapper.mapToDroneDto(updatedDrone);
    }

    @Override
    public void deleteDrone(Long id) {
        Drone drone = droneRepository.findById(id)
                .orElseThrow(() -> new ResourceNotFoundException("Drone not found with id: " + id));
        droneRepository.delete(drone);
    }
}