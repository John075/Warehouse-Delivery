package com.warehouse_delivery.spring_boot.mapper;

import com.warehouse_delivery.spring_boot.dto.DroneDto;
import com.warehouse_delivery.spring_boot.entity.Drone;

public class DroneMapper {
    public DroneMapper() {

    }

    public static DroneDto mapToDroneDto(final Drone drone) {
        final DroneDto droneDto = new DroneDto();

        droneDto.setId(drone.getId());
        droneDto.setName(drone.getName());

        return droneDto;
    }

    public static Drone mapToDrone(final DroneDto droneDto) {
        final Drone drone = new Drone();
        drone.setId(droneDto.getId());
        drone.setName(droneDto.getName());

        return drone;
    }
}
