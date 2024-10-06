package com.warehouse_delivery.spring_boot.controllers;

import com.warehouse_delivery.spring_boot.dto.WarehouseDto;
import com.warehouse_delivery.spring_boot.entity.Warehouse;
import com.warehouse_delivery.spring_boot.mapper.WarehouseMapper;
import com.warehouse_delivery.spring_boot.services.DroneModelService;
import com.warehouse_delivery.spring_boot.services.WarehouseService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequestMapping("/api/warehouse")
public class WarehouseController {

    private WarehouseService warehouseService;

    @Autowired
    public WarehouseController(final WarehouseService service) {
        this.warehouseService = service;
    }

    @GetMapping
    public List<WarehouseDto> getAllWarehouses() {
        return warehouseService.getAllWarehouses();
    }

    @GetMapping("/{id}")
    public ResponseEntity<WarehouseDto> getWarehouseById(@PathVariable Long id) {
        WarehouseDto warehouse = warehouseService.getWarehouseById(id);
        return ResponseEntity.ok(warehouse);
    }

    @PostMapping
    public ResponseEntity<WarehouseDto> createWarehouse(@RequestBody WarehouseDto warehouseDto) {
        WarehouseDto savedWarehouse = warehouseService.createWarehouse(warehouseDto);
        return ResponseEntity.ok(savedWarehouse);
    }

    @PutMapping("/{id}")
    public ResponseEntity<WarehouseDto> updateWarehouse(@PathVariable Long id, @RequestBody WarehouseDto warehouseDto) {
        WarehouseDto updatedWarehouse = warehouseService.updateWarehouse(id, warehouseDto);
        if (updatedWarehouse != null) {
            return ResponseEntity.ok(updatedWarehouse);
        } else {
            return ResponseEntity.notFound().build();
        }
    }

    @DeleteMapping("/{id}")
    public ResponseEntity<Void> deleteWarehouse(@PathVariable Long id) {
        warehouseService.deleteWarehouse(id);
        return ResponseEntity.noContent().build();
    }
}