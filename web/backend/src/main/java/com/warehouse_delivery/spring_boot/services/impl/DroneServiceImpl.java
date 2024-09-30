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

    final DroneRepository droneRepository;

    @Autowired
    public DroneServiceImpl(DroneRepository repository) {
        this.droneRepository = repository;
    }

    @Override
    public DroneDto getDrone(Long id) {
        final Drone drone = droneRepository.findById(id).orElseThrow(
                () -> new ResourceNotFoundException("Drone does not exist with id " + id));
        return DroneMapper.mapToDroneDto(drone);
    }

    @Override
    public List<DroneDto> getAllDrones() {
        final List<Drone> drones = droneRepository.findAll();
        return drones.stream().map(DroneMapper::mapToDroneDto).toList();
    }

    @Override
    public DroneDto registerDrone(DroneDto droneDto) {
        final Drone registeredDrone = droneRepository.save(DroneMapper.mapToDrone(droneDto));
        return DroneMapper.mapToDroneDto(registeredDrone);
    }
}
