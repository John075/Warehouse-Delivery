package com.warehouse_delivery.spring_boot.services;

import com.warehouse_delivery.spring_boot.dto.PackageDto;

import java.util.List;

public interface PackageService {
    PackageDto getPackage(Long id);
    List<PackageDto> getAllPackages();
    PackageDto registerPackage(PackageDto packageDto);
    PackageDto updatePackage(Long id, PackageDto packageDto);
    void deletePackage(Long id);
}