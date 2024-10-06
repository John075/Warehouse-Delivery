package com.warehouse_delivery.spring_boot.services.impl;

import com.warehouse_delivery.spring_boot.dto.PackageDto;
import com.warehouse_delivery.spring_boot.entity.Package;
import com.warehouse_delivery.spring_boot.mapper.PackageMapper;
import com.warehouse_delivery.spring_boot.messages.errors.ResourceNotFoundException;
import com.warehouse_delivery.spring_boot.repositories.PackageRepository;
import com.warehouse_delivery.spring_boot.services.PackageService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
public class PackageServiceImpl implements PackageService {

    final PackageRepository packageRepository;

    @Autowired
    public PackageServiceImpl(PackageRepository repository) {
        this.packageRepository = repository;
    }

    @Override
    public PackageDto getPackage(Long id) {
        final Package packageEntity = packageRepository.findById(id).orElseThrow(
                () -> new ResourceNotFoundException("package does not exist with id " + id));
        return PackageMapper.mapToPackageDto(packageEntity);
    }

    @Override
    public List<PackageDto> getAllPackages() {
        final List<Package> packages = packageRepository.findAll();
        return packages.stream().map(PackageMapper::mapToPackageDto).toList();
    }

    @Override
    public PackageDto registerPackage(PackageDto packageDto) {
        final Package registeredpackage = packageRepository.save(PackageMapper.mapToPackage(packageDto));
        return PackageMapper.mapToPackageDto(registeredpackage);
    }
}
