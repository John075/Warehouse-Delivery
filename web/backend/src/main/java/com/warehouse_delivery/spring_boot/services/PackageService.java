package com.warehouse_delivery.spring_boot.services;

import com.warehouse_delivery.spring_boot.dto.DroneDto;
import com.warehouse_delivery.spring_boot.dto.PackageDto;

import java.util.List;

public interface PackageService {
    PackageDto getPackage(final Long id);

    List<PackageDto> getAllPackages();

    PackageDto registerPackage(PackageDto droneDto);
}
