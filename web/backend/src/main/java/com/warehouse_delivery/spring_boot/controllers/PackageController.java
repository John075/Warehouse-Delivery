package com.warehouse_delivery.spring_boot.controllers;

import com.warehouse_delivery.spring_boot.dto.DroneDto;
import com.warehouse_delivery.spring_boot.dto.PackageDto;
import com.warehouse_delivery.spring_boot.services.DroneService;
import com.warehouse_delivery.spring_boot.services.PackageService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@CrossOrigin("*")
@RestController
@RequestMapping("/api/package")
public class PackageController {

    private PackageService packageService;

    @Autowired
    public PackageController(final PackageService service) {
        this.packageService = service;
    }

    @GetMapping("{id}")
    public ResponseEntity<PackageDto> getPackage(@PathVariable("id") final Long id) {
        final PackageDto packageDto = packageService.getPackage(id);
        return ResponseEntity.ok(packageDto);
    }

    @GetMapping
    public ResponseEntity<List<PackageDto>> getPackages() {
        final List<PackageDto> packageDtoList = packageService.getAllPackages();
        return ResponseEntity.ok(packageDtoList);
    }

    @PostMapping
    public ResponseEntity<PackageDto> registerPackage(@RequestBody final PackageDto packageDto) {
        final PackageDto newDto = packageService.registerPackage(packageDto);
        return ResponseEntity.ok(newDto);
    }

}
