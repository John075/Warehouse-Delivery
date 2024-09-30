package com.warehouse_delivery.spring_boot.controllers;

import com.warehouse_delivery.spring_boot.dto.DroneDto;
import com.warehouse_delivery.spring_boot.messages.GeneralResponse;
import com.warehouse_delivery.spring_boot.services.DroneService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@CrossOrigin("*")
@RestController
@RequestMapping("/api/drone")
public class DroneController {

    private DroneService droneService;

    @Autowired
    public DroneController(final DroneService service) {
        this.droneService = service;
    }

    @GetMapping("{id}")
    public ResponseEntity<DroneDto> getDrone(@PathVariable("id") final Long id) {
        final DroneDto droneDto = droneService.getDrone(id);
        return ResponseEntity.ok(droneDto);
    }

    @GetMapping
    public ResponseEntity<List<DroneDto>> getDrones() {
        final List<DroneDto> droneDtoList = droneService.getAllDrones();
        return ResponseEntity.ok(droneDtoList);
    }

    @PostMapping
    public ResponseEntity<DroneDto> registerDrone(@RequestBody final DroneDto droneDto) {
        final DroneDto newDto = droneService.registerDrone(droneDto);
        return ResponseEntity.ok(newDto);
    }

}
