package com.warehouse_delivery.spring_boot.services.impl;

import com.warehouse_delivery.spring_boot.broadcaster.DroneUpdateBroadcaster;
import com.warehouse_delivery.spring_boot.dto.PackageDto;
import com.warehouse_delivery.spring_boot.entity.Drone;
import com.warehouse_delivery.spring_boot.entity.Package;
import com.warehouse_delivery.spring_boot.mapper.DroneMapper;
import com.warehouse_delivery.spring_boot.mapper.PackageMapper;
import com.warehouse_delivery.spring_boot.messages.errors.ResourceNotFoundException;
import com.warehouse_delivery.spring_boot.repositories.DroneRepository;
import com.warehouse_delivery.spring_boot.repositories.PackageRepository;
import com.warehouse_delivery.spring_boot.services.PackageService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.Comparator;
import java.util.List;

@Service
public class PackageServiceImpl implements PackageService {

    final PackageRepository packageRepository;
    final DroneRepository droneRepository;
    final DroneUpdateBroadcaster broadcaster;

    @Autowired
    public PackageServiceImpl(DroneRepository droneRepository, PackageRepository packageRepository, DroneUpdateBroadcaster broadcaster) {
        this.droneRepository = droneRepository;
        this.packageRepository = packageRepository;
        this.broadcaster = broadcaster;
    }

    @Override
    public PackageDto getPackage(Long id) {
        final Package packageEntity = packageRepository.findById(id)
                .orElseThrow(() -> new ResourceNotFoundException("Package does not exist with id " + id));

        return PackageMapper.mapToPackageDto(packageEntity, true);
    }

    @Override
    public List<PackageDto> getAllPackages() {
        final List<Package> packages = packageRepository.findAll();
        packages.sort(Comparator.comparing(Package::getId));
        return packages.stream().map((packageEntity) -> PackageMapper.mapToPackageDto(packageEntity, true)).toList();
    }

    @Override
    public PackageDto registerPackage(PackageDto packageDto) {
        Package mappedEntity = PackageMapper.mapToPackage(packageDto, true);
        final Package savedPackage = packageRepository.save(mappedEntity);

        if (savedPackage.getAssignedDrone() != null) {
            Drone assignedDrone = droneRepository.findById(savedPackage.getAssignedDrone().getId())
                    .orElseThrow(() -> new ResourceNotFoundException("Drone not found with id: " + savedPackage.getAssignedDrone().getId()));

            assignedDrone.getPackages().add(savedPackage);
            droneRepository.save(assignedDrone);
        }

        return PackageMapper.mapToPackageDto(savedPackage, true);
    }

    @Override
    public PackageDto updatePackage(Long id, PackageDto packageDto) {
        packageRepository.findById(id)
                .orElseThrow(() -> new ResourceNotFoundException("Package does not exist with id " + id));

        Package updatedPackage = packageRepository.save(PackageMapper.mapToPackage(packageDto, true));

        if (packageDto.getAssignedDrone() != null) {
            Drone assignedDrone = droneRepository.findById(updatedPackage.getAssignedDrone().getId())
                    .orElseThrow(() -> new ResourceNotFoundException("Drone not found with id: " + updatedPackage.getAssignedDrone().getId()));

            boolean packageExists = assignedDrone.getPackages()
                    .stream()
                    .anyMatch(pkg -> pkg.getId().equals(updatedPackage.getId()));

            if (!packageExists) {
                assignedDrone.getPackages().add(updatedPackage);
                droneRepository.save(assignedDrone);
                broadcaster.broadcastUpdate(assignedDrone.getId(), DroneMapper.mapToDroneDto(assignedDrone, true));
            }
        }
        return PackageMapper.mapToPackageDto(updatedPackage, true);
    }
    @Override
    public void deletePackage(Long id) {
        Package packageEntity = packageRepository.findById(id)
                .orElseThrow(() -> new ResourceNotFoundException("Package does not exist with id " + id));
        packageRepository.delete(packageEntity);
    }
}