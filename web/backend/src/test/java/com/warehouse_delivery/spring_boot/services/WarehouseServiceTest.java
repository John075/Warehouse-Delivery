package com.warehouse_delivery.spring_boot.services;

import com.warehouse_delivery.spring_boot.dto.WarehouseDto;
import com.warehouse_delivery.spring_boot.entity.Address;
import com.warehouse_delivery.spring_boot.entity.Warehouse;
import com.warehouse_delivery.spring_boot.mapper.WarehouseMapper;
import com.warehouse_delivery.spring_boot.messages.errors.ResourceNotFoundException;
import com.warehouse_delivery.spring_boot.repositories.WarehouseRepository;
import com.warehouse_delivery.spring_boot.services.impl.WarehouseServiceImpl;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyLong;
import static org.mockito.Mockito.*;

class WarehouseServiceTest {

    @Mock
    private WarehouseRepository warehouseRepository;

    @InjectMocks
    private WarehouseServiceImpl warehouseService;

    private Warehouse testWarehouse;
    private WarehouseDto testWarehouseDto;

    @BeforeEach
    void setUp() {
        MockitoAnnotations.openMocks(this);

        // Initialize test warehouse and warehouse DTO
        Address address = new Address("123 Street", "City", "State", "12345", "Country");
        testWarehouse = new Warehouse("Main Warehouse", 40.7128, -74.0060, address, 1000);
        testWarehouse.setId(1L);

        testWarehouseDto = WarehouseMapper.mapToWarehouseDto(testWarehouse);
    }

    @Test
    void testGetAllWarehouses() {
        List<Warehouse> warehouses = new ArrayList<>();
        warehouses.add(testWarehouse);

        when(warehouseRepository.findAll()).thenReturn(warehouses);

        List<WarehouseDto> warehouseDtos = warehouseService.getAllWarehouses();

        assertNotNull(warehouseDtos);
        assertEquals(1, warehouseDtos.size());
        assertEquals("Main Warehouse", warehouseDtos.get(0).getName());
        assertEquals(40.7128, warehouseDtos.get(0).getLatitude());

        verify(warehouseRepository, times(1)).findAll();
    }

    @Test
    void testGetWarehouseByIdSuccess() {
        when(warehouseRepository.findById(anyLong())).thenReturn(Optional.of(testWarehouse));

        WarehouseDto result = warehouseService.getWarehouseById(1L);

        assertNotNull(result);
        assertEquals(1L, result.getId());
        assertEquals("Main Warehouse", result.getName());
        assertEquals(40.7128, result.getLatitude());

        verify(warehouseRepository, times(1)).findById(1L);
    }

    @Test
    void testGetWarehouseByIdNotFound() {
        when(warehouseRepository.findById(anyLong())).thenReturn(Optional.empty());

        assertThrows(ResourceNotFoundException.class, () -> warehouseService.getWarehouseById(1L));

        verify(warehouseRepository, times(1)).findById(1L);
    }

    @Test
    void testCreateWarehouse() {
        when(warehouseRepository.save(any(Warehouse.class))).thenReturn(testWarehouse);

        WarehouseDto result = warehouseService.createWarehouse(testWarehouseDto);

        assertNotNull(result);
        assertEquals(1L, result.getId());
        assertEquals("Main Warehouse", result.getName());

        verify(warehouseRepository, times(1)).save(any(Warehouse.class));
    }

    @Test
    void testUpdateWarehouseSuccess() {
        when(warehouseRepository.findById(anyLong())).thenReturn(Optional.of(testWarehouse));

        testWarehouseDto.setName("Updated Warehouse");
        testWarehouseDto.setCapacity(1200);

        when(warehouseRepository.save(any(Warehouse.class))).thenReturn(WarehouseMapper.mapToWarehouse(testWarehouseDto));

        WarehouseDto result = warehouseService.updateWarehouse(1L, testWarehouseDto);

        assertNotNull(result);
        assertEquals(1L, result.getId());
        assertEquals("Updated Warehouse", result.getName());
        assertEquals(1200, result.getCapacity());

        verify(warehouseRepository, times(1)).findById(1L);
        verify(warehouseRepository, times(1)).save(any(Warehouse.class));
    }

    @Test
    void testDeleteWarehouseSuccess() {
        when(warehouseRepository.findById(anyLong())).thenReturn(Optional.of(testWarehouse));

        warehouseService.deleteWarehouse(1L);

        verify(warehouseRepository, times(1)).findById(1L);
        verify(warehouseRepository, times(1)).delete(any(Warehouse.class));
    }

    @Test
    void testDeleteWarehouseNotFound() {
        when(warehouseRepository.findById(anyLong())).thenReturn(Optional.empty());

        assertThrows(ResourceNotFoundException.class, () -> warehouseService.deleteWarehouse(1L));

        verify(warehouseRepository, times(1)).findById(1L);
    }
}