package com.warehouse_delivery.spring_boot.mapper;

import com.warehouse_delivery.spring_boot.dto.PackageDto;
import com.warehouse_delivery.spring_boot.entity.Package;

public class PackageMapper {
    public PackageMapper() {

    }

    public static PackageDto mapToPackageDto(final Package drone) {
        final PackageDto packageDto = new PackageDto();

        packageDto.setId(drone.getId());
        packageDto.setName(drone.getName());

        return packageDto;
    }

    public static Package mapToPackage(final PackageDto droneDto) {
        final Package packageEntity = new Package();
        packageEntity.setId(droneDto.getId());
        packageEntity.setName(droneDto.getName());

        return packageEntity;
    }
}
