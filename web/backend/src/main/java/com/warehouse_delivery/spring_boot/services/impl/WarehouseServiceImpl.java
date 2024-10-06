package com.warehouse_delivery.spring_boot.services.impl;

import com.warehouse_delivery.spring_boot.dto.WarehouseDto;
import com.warehouse_delivery.spring_boot.entity.Warehouse;
import com.warehouse_delivery.spring_boot.mapper.WarehouseMapper;
import com.warehouse_delivery.spring_boot.messages.errors.ResourceNotFoundException;
import com.warehouse_delivery.spring_boot.repositories.WarehouseRepository;
import com.warehouse_delivery.spring_boot.services.WarehouseService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
public class WarehouseServiceImpl implements WarehouseService {

    @Autowired
    private WarehouseRepository warehouseRepository;

    /**
     * Retrieve all warehouses from the repository.
     *
     * @return List of Warehouse entities.
     */
    public List<WarehouseDto> getAllWarehouses() {
        List<Warehouse> warehouses = warehouseRepository.findAll();
        return warehouses.stream().map(WarehouseMapper::mapToWarehouseDto).toList();
    }

    /**
     * Retrieve a warehouse by its ID.
     *
     * @param id The ID of the warehouse.
     * @return The Warehouse entity or null if not found.
     */
    public WarehouseDto getWarehouseById(Long id) {
        Warehouse warehouse = warehouseRepository.findById(id).orElseThrow(() ->
            new ResourceNotFoundException("Warehouse does not exist with id " + id)
        );
        return WarehouseMapper.mapToWarehouseDto(warehouse);
    }

    /**
     * Create a new warehouse.
     *
     * @param warehouse The Warehouse entity to be saved.
     * @return The saved Warehouse entity.
     */
    public WarehouseDto createWarehouse(WarehouseDto warehouse) {
        Warehouse updatedWarehouse = warehouseRepository.save(WarehouseMapper.mapToWarehouse(warehouse));
        return WarehouseMapper.mapToWarehouseDto(updatedWarehouse);
    }

    /**
     * Update an existing warehouse by its ID.
     *
     * @param id The ID of the warehouse to update.
     * @param warehouseDto The new warehouse data.
     * @return The updated Warehouse entity or null if not found.
     */
    @Override
    public WarehouseDto updateWarehouse(Long id, WarehouseDto warehouseDto) {
        warehouseRepository.findById(id)
                .orElseThrow(() -> new ResourceNotFoundException("Warehouse does not exist with id " + id));

        // Save the updated warehouse
        Warehouse updatedWarehouse = warehouseRepository.save(WarehouseMapper.mapToWarehouse(warehouseDto));
        return WarehouseMapper.mapToWarehouseDto(updatedWarehouse);
    }

    /**
     * Delete a warehouse by its ID.
     *
     * @param id The ID of the warehouse to delete.
     */
    public void deleteWarehouse(Long id) {
        Warehouse warehouse = warehouseRepository.findById(id).orElseThrow(() ->
                new ResourceNotFoundException("Warehouse does not exist with id " + id)
        );
        warehouseRepository.delete(warehouse);
    }
}