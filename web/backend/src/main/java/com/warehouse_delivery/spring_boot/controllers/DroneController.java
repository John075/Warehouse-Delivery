package com.warehouse_delivery.spring_boot.controllers;

import com.warehouse_delivery.spring_boot.broadcaster.DroneUpdateBroadcaster;
import com.warehouse_delivery.spring_boot.dto.DroneDto;
import com.warehouse_delivery.spring_boot.services.DroneService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;
import org.springframework.web.servlet.mvc.method.annotation.SseEmitter;

import java.util.List;

@CrossOrigin("*")
@RestController
@RequestMapping("/api/drone")
public class DroneController {

    private DroneService droneService;
    private DroneUpdateBroadcaster broadcaster;

    @Autowired
    public DroneController(final DroneService service, final DroneUpdateBroadcaster broadcaster) {
        this.droneService = service;
        this.broadcaster = broadcaster;
    }

    /**
     * Get a specific drone by ID.
     *
     * @param id The ID of the drone.
     * @return ResponseEntity containing the drone data.
     */
    @GetMapping("{id}")
    public ResponseEntity<DroneDto> getDrone(@PathVariable("id") final Long id) {
        final DroneDto droneDto = droneService.getDrone(id);
        return ResponseEntity.ok(droneDto);
    }

    /**
     * Get all drones.
     *
     * @return ResponseEntity containing the list of all drones.
     */
    @GetMapping
    public ResponseEntity<List<DroneDto>> getDrones() {
        final List<DroneDto> droneDtoList = droneService.getAllDrones();
        return ResponseEntity.ok(droneDtoList);
    }

    /**
     * Register a new drone.
     *
     * @param droneDto The drone data to be registered.
     * @return ResponseEntity containing the registered drone.
     */
    @PostMapping
    public ResponseEntity<DroneDto> registerDrone(@RequestBody final DroneDto droneDto) {
        final DroneDto newDto = droneService.registerDrone(droneDto);
        return ResponseEntity.ok(newDto);
    }

    /**
     * Update an existing drone by ID.
     *
     * @param id       The ID of the drone to be updated.
     * @param droneDto The new data for the drone.
     * @return ResponseEntity containing the updated drone.
     */
    @PutMapping("{id}")
    public ResponseEntity<DroneDto> updateDrone(@PathVariable("id") final Long id, @RequestBody final DroneDto droneDto) {
        final DroneDto updatedDto = droneService.updateDrone(id, droneDto);
        return ResponseEntity.ok(updatedDto);
    }

    /**
     * Delete a drone by ID.
     *
     * @param id The ID of the drone to be deleted.
     * @return ResponseEntity with no content.
     */
    @DeleteMapping("{id}")
    public ResponseEntity<Void> deleteDrone(@PathVariable("id") final Long id) {
        droneService.deleteDrone(id);
        return ResponseEntity.noContent().build();
    }

    /**
     * Provide a SseEmitter to the client to provide live server-provided updates when the entity changes
     *
     * @param id The ID of the drone to be connected to
     * @return SseEmitter with drone related info to the ID requested
     */
    @GetMapping(value = "{id}/updates", produces = MediaType.TEXT_EVENT_STREAM_VALUE)
    public SseEmitter streamDroneUpdates(@PathVariable("id") final Long id) {
        return broadcaster.registerEmitter(id, droneService.getDrone(id));
    }
}