package com.warehouse_delivery.spring_boot.services;

import com.warehouse_delivery.spring_boot.dto.DroneDto;
import com.warehouse_delivery.spring_boot.repositories.DroneRepository;
import org.springframework.stereotype.Service;

import java.util.List;

public interface DroneService {
    DroneDto getDrone(final Long id);

    List<DroneDto> getAllDrones();

    DroneDto registerDrone(DroneDto droneDto);
}
