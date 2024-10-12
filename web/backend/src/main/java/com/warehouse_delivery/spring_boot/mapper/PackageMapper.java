package com.warehouse_delivery.spring_boot.mapper;

import com.warehouse_delivery.spring_boot.dto.PackageDto;
import com.warehouse_delivery.spring_boot.entity.Package;

public class PackageMapper {

    public PackageMapper() {
    }

    public static PackageDto mapToPackageDto(final Package packageEntity, boolean includeDroneDetails) {
        final PackageDto packageDto = new PackageDto();

        packageDto.setId(packageEntity.getId());
        packageDto.setName(packageEntity.getName());
        packageDto.setStatus(packageEntity.getStatus());
        packageDto.setDestination(packageEntity.getDestination());
        packageDto.setPriority(packageEntity.getPriority());
        packageDto.setOrderTime(packageEntity.getOrderTime());

        if (packageEntity.getAssignedDrone() != null && includeDroneDetails) {
            packageDto.setAssignedDrone(DroneMapper.mapToDroneDto(packageEntity.getAssignedDrone(), false));
        }

        if (packageEntity.getWarehouse() != null) {
            packageDto.setWarehouse(WarehouseMapper.mapToWarehouseDto(packageEntity.getWarehouse()));
        }

        return packageDto;
    }

    public static Package mapToPackage(final PackageDto packageDto, boolean includeDroneDetails) {
        final Package packageEntity = new Package();

        packageEntity.setId(packageDto.getId());
        packageEntity.setName(packageDto.getName());
        packageEntity.setStatus(packageDto.getStatus());
        packageEntity.setDestination(packageDto.getDestination());
        packageEntity.setPriority(packageDto.getPriority());
        packageEntity.setOrderTime(packageDto.getOrderTime());

        if (packageDto.getAssignedDrone() != null && includeDroneDetails) {
            packageEntity.setAssignedDrone(DroneMapper.mapToDrone(packageDto.getAssignedDrone(), false));
        }

        if (packageDto.getWarehouse() != null) {
            packageEntity.setWarehouse(WarehouseMapper.mapToWarehouse(packageDto.getWarehouse()));
        }

        return packageEntity;
    }
}