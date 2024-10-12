package com.warehouse_delivery.spring_boot.services.impl;

import com.warehouse_delivery.spring_boot.dto.DroneDto;
import com.warehouse_delivery.spring_boot.entity.Drone;
import com.warehouse_delivery.spring_boot.mapper.DroneMapper;
import com.warehouse_delivery.spring_boot.messages.errors.ResourceNotFoundException;
import com.warehouse_delivery.spring_boot.repositories.DroneRepository;
import com.warehouse_delivery.spring_boot.services.DroneService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.Comparator;
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

        return DroneMapper.mapToDroneDto(drone, true);
    }

    @Override
    public List<DroneDto> getAllDrones() {
        List<Drone> drones = droneRepository.findAll();
        drones.sort(Comparator.comparing(Drone::getId));
        return drones.stream().map((droneDto) -> DroneMapper.mapToDroneDto(droneDto, true)).toList();
    }

    @Override
    public DroneDto registerDrone(DroneDto droneDto) {
        Drone newDrone = DroneMapper.mapToDrone(droneDto, true);
        droneRepository.save(newDrone);
        return DroneMapper.mapToDroneDto(newDrone, true);
    }

    @Override
    public DroneDto updateDrone(Long id, DroneDto droneDto) {
        droneRepository.findById(id)
                .orElseThrow(() -> new ResourceNotFoundException("Drone not found with id: " + id));

        Drone mappedDrone = DroneMapper.mapToDrone(droneDto, true);
        Drone updatedDrone = droneRepository.save(mappedDrone);

        return DroneMapper.mapToDroneDto(updatedDrone, true);
    }

    @Override
    public void deleteDrone(Long id) {
        Drone drone = droneRepository.findById(id)
                .orElseThrow(() -> new ResourceNotFoundException("Drone not found with id: " + id));
        droneRepository.delete(drone);
    }
}