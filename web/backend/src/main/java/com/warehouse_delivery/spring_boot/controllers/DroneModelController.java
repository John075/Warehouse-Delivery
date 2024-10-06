package com.warehouse_delivery.spring_boot.controllers;

import com.warehouse_delivery.spring_boot.dto.DroneModelDto;
import com.warehouse_delivery.spring_boot.entity.DroneModel;
import com.warehouse_delivery.spring_boot.mapper.DroneModelMapper;
import com.warehouse_delivery.spring_boot.services.DroneModelService;
import com.warehouse_delivery.spring_boot.services.PackageService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequestMapping("/api/drone-model")
public class DroneModelController {

    private DroneModelService droneModelService;

    @Autowired
    public DroneModelController(final DroneModelService service) {
        this.droneModelService = service;
    }

    @GetMapping
    public List<DroneModelDto> getAllDroneModels() {
        return droneModelService.getAllDroneModels();
    }

    @GetMapping("/{id}")
    public ResponseEntity<DroneModelDto> getDroneModelById(@PathVariable Long id) {
        DroneModelDto droneModel = droneModelService.getDroneModelById(id);
        return ResponseEntity.ok(droneModel);
    }

    @PostMapping
    public ResponseEntity<DroneModelDto> createDroneModel(@RequestBody DroneModelDto droneModelDto) {
        DroneModelDto savedDroneModel = droneModelService.createDroneModel(droneModelDto);
        return ResponseEntity.ok(savedDroneModel);
    }

    @PutMapping("/{id}")
    public ResponseEntity<DroneModelDto> updateDroneModel(@PathVariable Long id, @RequestBody DroneModelDto droneModelDto) {
        DroneModelDto updatedDroneModel = droneModelService.updateDroneModel(id, droneModelDto);
        return ResponseEntity.ok(updatedDroneModel);
    }

    @DeleteMapping("/{id}")
    public ResponseEntity<Void> deleteDroneModel(@PathVariable Long id) {
        droneModelService.deleteDroneModel(id);
        return ResponseEntity.noContent().build();
    }
}