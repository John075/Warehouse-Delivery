package com.warehouse_delivery.spring_boot.controllers;

import com.warehouse_delivery.spring_boot.dto.PackageDto;
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

    /**
     * Get a package by ID.
     *
     * @param id The ID of the package.
     * @return A response containing the package details.
     */
    @GetMapping("{id}")
    public ResponseEntity<PackageDto> getPackage(@PathVariable("id") final Long id) {
        final PackageDto packageDto = packageService.getPackage(id);
        return ResponseEntity.ok(packageDto);
    }

    /**
     * Get all packages.
     *
     * @return A response containing a list of all packages.
     */
    @GetMapping
    public ResponseEntity<List<PackageDto>> getPackages() {
        final List<PackageDto> packageDtoList = packageService.getAllPackages();
        return ResponseEntity.ok(packageDtoList);
    }

    /**
     * Register a new package.
     *
     * @param packageDto The package to be registered.
     * @return A response containing the registered package details.
     */
    @PostMapping
    public ResponseEntity<PackageDto> registerPackage(@RequestBody final PackageDto packageDto) {
        final PackageDto newDto = packageService.registerPackage(packageDto);
        return ResponseEntity.ok(newDto);
    }

    /**
     * Update an existing package by ID.
     *
     * @param id The ID of the package to be updated.
     * @param packageDto The new package details to be updated.
     * @return A response containing the updated package details.
     */
    @PutMapping("{id}")
    public ResponseEntity<PackageDto> updatePackage(@PathVariable("id") final Long id, @RequestBody final PackageDto packageDto) {
        final PackageDto updatedPackage = packageService.updatePackage(id, packageDto);
        return ResponseEntity.ok(updatedPackage);
    }

    /**
     * Delete a package by ID.
     *
     * @param id The ID of the package to be deleted.
     * @return A response confirming the deletion.
     */
    @DeleteMapping("{id}")
    public ResponseEntity<Void> deletePackage(@PathVariable("id") final Long id) {
        packageService.deletePackage(id);
        return ResponseEntity.noContent().build();
    }
}