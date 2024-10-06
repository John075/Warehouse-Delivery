package com.warehouse_delivery.spring_boot.mapper;

import com.warehouse_delivery.spring_boot.dto.DroneModelDto;
import com.warehouse_delivery.spring_boot.entity.DroneModel;

public class DroneModelMapper {

    /**
     * Maps a DroneModel entity to a DroneModelDto.
     *
     * @param droneModel The DroneModel entity to be mapped.
     * @return A DroneModelDto containing the mapped data.
     */
    public static DroneModelDto mapToDroneModelDto(DroneModel droneModel) {
        if (droneModel == null) {
            return null;
        }

        DroneModelDto droneModelDto = new DroneModelDto();
        droneModelDto.setId(droneModel.getId());
        droneModelDto.setManufacturer(droneModel.getManufacturer());
        droneModelDto.setMaxSpeed(droneModel.getMaxSpeed());
        droneModelDto.setRecommendedSpeed(droneModel.getRecommendedSpeed());
        droneModelDto.setBatteryCapacity(droneModel.getBatteryCapacity());
        droneModelDto.setMaxTripLength(droneModel.getMaxTripLength());

        return droneModelDto;
    }

    /**
     * Maps a DroneModelDto to a DroneModel entity.
     *
     * @param droneModelDto The DroneModelDto to be mapped.
     * @return A DroneModel entity containing the mapped data.
     */
    public static DroneModel mapToDroneModel(DroneModelDto droneModelDto) {
        if (droneModelDto == null) {
            return null;
        }

        DroneModel droneModel = new DroneModel();
        droneModel.setId(droneModelDto.getId());
        droneModel.setManufacturer(droneModelDto.getManufacturer());
        droneModel.setMaxSpeed(droneModelDto.getMaxSpeed());
        droneModel.setRecommendedSpeed(droneModelDto.getRecommendedSpeed());
        droneModel.setBatteryCapacity(droneModelDto.getBatteryCapacity());
        droneModel.setMaxTripLength(droneModelDto.getMaxTripLength());

        return droneModel;
    }
}