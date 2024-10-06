package com.warehouse_delivery.spring_boot.services;

import com.warehouse_delivery.spring_boot.dto.WarehouseDto;

import java.util.List;

public interface WarehouseService {
    List<WarehouseDto> getAllWarehouses();
    WarehouseDto getWarehouseById(Long id);
    WarehouseDto createWarehouse(WarehouseDto warehouse);
    WarehouseDto updateWarehouse(Long id, WarehouseDto updatedWarehouse);
    void deleteWarehouse(Long id);
}
