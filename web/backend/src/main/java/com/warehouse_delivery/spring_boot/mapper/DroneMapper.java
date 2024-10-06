package com.warehouse_delivery.spring_boot.mapper;

import com.warehouse_delivery.spring_boot.dto.DroneDto;
import com.warehouse_delivery.spring_boot.entity.Drone;

import java.util.stream.Collectors;

public class DroneMapper {
    public DroneMapper() {

    }

    public static DroneDto mapToDroneDto(final Drone drone) {
        final DroneDto droneDto = new DroneDto();

        droneDto.setId(drone.getId());
        droneDto.setName(drone.getName());
        droneDto.setStatus(drone.getStatus());
        droneDto.setBatteryLife(drone.getBatteryLife());
        droneDto.setLastLatitudeLocation(drone.getLastLatitudeLocation());
        droneDto.setLastLongitudeLocation(drone.getLastLongitudeLocation());
        droneDto.setConnectedToSystem(drone.isConnectedToSystem());
        droneDto.setLastMaintenanceDate(drone.getLastMaintenanceDate());

        if (drone.getPackages() != null) {
            droneDto.setPackages(
                    drone.getPackages().stream().map(PackageMapper::mapToPackageDto).collect(Collectors.toList())
            );
        }

        if(drone.getWarehouse() != null) {
            droneDto.setWarehouse(WarehouseMapper.mapToWarehouseDto(drone.getWarehouse()));
        }

        return droneDto;
    }

    public static Drone mapToDrone(final DroneDto droneDto) {
        final Drone drone = new Drone();

        drone.setId(droneDto.getId());
        drone.setName(droneDto.getName());
        drone.setStatus(droneDto.getStatus());
        drone.setBatteryLife(droneDto.getBatteryLife());
        drone.setLastLatitudeLocation(droneDto.getLastLatitudeLocation());
        drone.setLastLongitudeLocation(droneDto.getLastLongitudeLocation());
        drone.setConnectedToSystem(droneDto.isConnectedToSystem());
        drone.setLastMaintenanceDate(droneDto.getLastMaintenanceDate());

        if (droneDto.getPackages() != null) {
            drone.setPackages(
                    droneDto.getPackages().stream().map(PackageMapper::mapToPackage).collect(Collectors.toList())
            );
        }

        if(droneDto.getWarehouse() != null) {
            drone.setWarehouse(WarehouseMapper.mapToWarehouse(droneDto.getWarehouse()));
        }

        return drone;
    }
}
