package com.warehouse_delivery.spring_boot.services;

import com.warehouse_delivery.spring_boot.dto.DroneDto;
import com.warehouse_delivery.spring_boot.entity.Drone;
import com.warehouse_delivery.spring_boot.entity.DroneModel;
import com.warehouse_delivery.spring_boot.entity.Warehouse;
import com.warehouse_delivery.spring_boot.enums.DroneStatus;
import com.warehouse_delivery.spring_boot.mapper.DroneMapper;
import com.warehouse_delivery.spring_boot.messages.errors.ResourceNotFoundException;
import com.warehouse_delivery.spring_boot.repositories.DroneRepository;
import com.warehouse_delivery.spring_boot.services.impl.DroneServiceImpl;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import java.time.LocalDateTime;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyLong;
import static org.mockito.Mockito.*;

class DroneServiceTest {

    @Mock
    private DroneRepository droneRepository;

    @InjectMocks
    private DroneServiceImpl droneService;

    private Drone testDrone;
    private DroneDto testDroneDto;

    @BeforeEach
    public void setUp() {
        MockitoAnnotations.openMocks(this);

        testDrone = new Drone();
        testDrone.setId(1L);
        testDrone.setName("Delivery Drone A");
        testDrone.setStatus(DroneStatus.IN_TRANSIT);
        testDrone.setLastLatitudeLocation(37.7749);
        testDrone.setLastLongitudeLocation(-122.4194);
        testDrone.setBatteryLife(75.0);
        testDrone.setLastMaintenanceDate(LocalDateTime.now());
        testDrone.setConnectedToSystem(true);

        DroneModel droneModel = new DroneModel();
        droneModel.setId(1L);
        droneModel.setManufacturer("DJI");
        testDrone.setDroneModel(droneModel);

        Warehouse warehouse = new Warehouse();
        warehouse.setId(1L);
        warehouse.setName("Main Warehouse");
        testDrone.setWarehouse(warehouse);

        testDroneDto = DroneMapper.mapToDroneDto(testDrone, true);
    }

    @Test
    public void testGetDroneSuccess() {
        when(droneRepository.findById(anyLong())).thenReturn(Optional.of(testDrone));

        DroneDto result = droneService.getDrone(1L);

        assertNotNull(result);
        assertEquals(1L, result.getId());
        assertEquals("Delivery Drone A", result.getName());
        assertEquals(DroneStatus.IN_TRANSIT, result.getStatus());
        assertEquals(75.0, result.getBatteryLife());
        assertEquals("Main Warehouse", result.getWarehouse().getName());

        verify(droneRepository, times(1)).findById(1L);
    }

    @Test
    public void testGetDroneNotFound() {
        when(droneRepository.findById(anyLong())).thenReturn(Optional.empty());

        assertThrows(ResourceNotFoundException.class, () -> droneService.getDrone(1L));
        verify(droneRepository, times(1)).findById(1L);
    }

    @Test
    public void testGetAllDrones() {
        Drone secondDrone = new Drone();
        secondDrone.setId(2L);
        secondDrone.setName("Delivery Drone B");

        List<Drone> droneList = Arrays.asList(testDrone, secondDrone);
        when(droneRepository.findAll()).thenReturn(droneList);

        List<DroneDto> result = droneService.getAllDrones();

        assertNotNull(result);
        assertEquals(2, result.size());
        assertEquals(1L, result.get(0).getId());
        assertEquals(2L, result.get(1).getId());

        verify(droneRepository, times(1)).findAll();
    }

    @Test
    public void testRegisterDrone() {
        when(droneRepository.save(any(Drone.class))).thenReturn(testDrone);

        DroneDto result = droneService.registerDrone(testDroneDto);

        assertNotNull(result);
        assertEquals(1L, result.getId());
        assertEquals("Delivery Drone A", result.getName());

        verify(droneRepository, times(1)).save(any(Drone.class));
    }

    @Test
    public void testUpdateDrone_Success() {
        testDroneDto.setName("Updated Drone A");
        testDroneDto.setStatus(DroneStatus.GOING_TO_WAREHOUSE);
        testDroneDto.setBatteryLife(80.0);

        when(droneRepository.findById(anyLong())).thenReturn(Optional.of(testDrone));
        when(droneRepository.save(any())).thenReturn(DroneMapper.mapToDrone(testDroneDto, true));

        DroneDto result = droneService.updateDrone(1L, testDroneDto);

        assertNotNull(result);
        assertEquals(1L, result.getId());
        assertEquals("Updated Drone A", result.getName());
        assertEquals(DroneStatus.GOING_TO_WAREHOUSE, result.getStatus());
        assertEquals(80.0, result.getBatteryLife(), 0.01);

        verify(droneRepository, times(1)).findById(1L);
        verify(droneRepository, times(1)).save(any(Drone.class));
    }
    @Test
    public void testUpdateDroneNotFound() {
        when(droneRepository.findById(anyLong())).thenReturn(Optional.empty());

        assertThrows(ResourceNotFoundException.class, () -> droneService.updateDrone(1L, testDroneDto));

        verify(droneRepository, times(1)).findById(1L);
    }

    @Test
    public void testDeleteDroneSuccess() {
        when(droneRepository.findById(anyLong())).thenReturn(Optional.of(testDrone));

        droneService.deleteDrone(1L);

        verify(droneRepository, times(1)).findById(1L);
        verify(droneRepository, times(1)).delete(testDrone);
    }

    @Test
    public void testDeleteDroneNotFound() {
        when(droneRepository.findById(anyLong())).thenReturn(Optional.empty());

        assertThrows(ResourceNotFoundException.class, () -> droneService.deleteDrone(1L));

        verify(droneRepository, times(1)).findById(1L);
    }
}