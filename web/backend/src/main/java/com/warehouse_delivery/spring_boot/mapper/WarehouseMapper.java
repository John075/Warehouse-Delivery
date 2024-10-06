package com.warehouse_delivery.spring_boot.mapper;

import com.warehouse_delivery.spring_boot.dto.WarehouseDto;
import com.warehouse_delivery.spring_boot.entity.Warehouse;

public class WarehouseMapper {

    public WarehouseMapper() {}

    public static WarehouseDto mapToWarehouseDto(final Warehouse warehouse) {
        WarehouseDto warehouseDto = new WarehouseDto();

        warehouseDto.setId(warehouse.getId());
        warehouseDto.setName(warehouse.getName());
        warehouseDto.setLatitude(warehouse.getLatitude());
        warehouseDto.setLongitude(warehouse.getLongitude());
        warehouseDto.setAddress(warehouse.getAddress());
        warehouseDto.setCapacity(warehouse.getCapacity());

        return warehouseDto;
    }

    public static Warehouse mapToWarehouse(final WarehouseDto warehouseDto) {
        Warehouse warehouse = new Warehouse();

        warehouse.setId(warehouseDto.getId());
        warehouse.setName(warehouseDto.getName());
        warehouse.setLatitude(warehouseDto.getLatitude());
        warehouse.setLongitude(warehouseDto.getLongitude());
        warehouse.setAddress(warehouseDto.getAddress());
        warehouse.setCapacity(warehouseDto.getCapacity());

        return warehouse;
    }
}