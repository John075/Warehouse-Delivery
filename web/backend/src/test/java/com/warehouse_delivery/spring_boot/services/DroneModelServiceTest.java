package com.warehouse_delivery.spring_boot.services;

import com.warehouse_delivery.spring_boot.dto.DroneModelDto;
import com.warehouse_delivery.spring_boot.entity.DroneModel;
import com.warehouse_delivery.spring_boot.entity.Warehouse;
import com.warehouse_delivery.spring_boot.mapper.DroneModelMapper;
import com.warehouse_delivery.spring_boot.messages.errors.ResourceNotFoundException;
import com.warehouse_delivery.spring_boot.repositories.DroneModelRepository;
import com.warehouse_delivery.spring_boot.services.DroneModelService;
import com.warehouse_delivery.spring_boot.services.impl.DroneModelServiceImpl;
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

public class DroneModelServiceTest {

    @Mock
    private DroneModelRepository droneModelRepository;

    @InjectMocks
    private DroneModelServiceImpl droneModelService;

    private DroneModel testDroneModel;
    private DroneModelDto testDroneModelDto;

    @BeforeEach
    public void setUp() {
        MockitoAnnotations.openMocks(this);

        testDroneModelDto = new DroneModelDto(1L, "DJI", 60.5, 50.0, 4000, 300);
        testDroneModel = DroneModelMapper.mapToDroneModel(testDroneModelDto);
    }

    @Test
    public void testGetAllDroneModels() {
        List<DroneModel> droneModels = new ArrayList<>();
        droneModels.add(testDroneModel);

        when(droneModelRepository.findAll()).thenReturn(droneModels);

        List<DroneModelDto> result = droneModelService.getAllDroneModels();

        assertNotNull(result);
        assertEquals(1, result.size());
        assertEquals("DJI", result.get(0).getManufacturer());

        verify(droneModelRepository, times(1)).findAll();
    }

    @Test
    public void testGetDroneModelById() {
        when(droneModelRepository.findById(anyLong())).thenReturn(Optional.of(testDroneModel));

        DroneModelDto result = droneModelService.getDroneModelById(1L);

        assertNotNull(result);
        assertEquals(1L, result.getId());
        assertEquals("DJI", result.getManufacturer());

        verify(droneModelRepository, times(1)).findById(1L);
    }

    @Test
    public void testGetDroneModelByIdNotFound() {
        when(droneModelRepository.findById(anyLong())).thenReturn(Optional.empty());

        assertThrows(ResourceNotFoundException.class, () -> {
            droneModelService.getDroneModelById(999L);
        });

        verify(droneModelRepository, times(1)).findById(999L);
    }

    @Test
    public void testCreateDroneModel() {
        when(droneModelRepository.save(any(DroneModel.class))).thenReturn(testDroneModel);

        DroneModelDto result = droneModelService.createDroneModel(testDroneModelDto);

        assertNotNull(result);
        assertEquals("DJI", result.getManufacturer());

        verify(droneModelRepository, times(1)).save(any(DroneModel.class));
    }

    @Test
    public void testUpdateDroneModel() {
        when(droneModelRepository.findById(anyLong())).thenReturn(Optional.of(testDroneModel));
        when(droneModelRepository.save(any(DroneModel.class))).thenAnswer(invocation -> invocation.getArgument(0));

        DroneModelDto updatedDto = new DroneModelDto(1L, "Parrot", 65.0, 55.0, 5000, 350);
        DroneModelDto result = droneModelService.updateDroneModel(1L, updatedDto);

        assertNotNull(result);

        assertEquals("Parrot", result.getManufacturer());
        assertEquals(65.0, result.getMaxSpeed());
        assertEquals(55.0, result.getRecommendedSpeed());
        assertEquals(5000, result.getBatteryCapacity());
        assertEquals(350, result.getMaxTripLength());

        verify(droneModelRepository, times(1)).findById(1L);
        verify(droneModelRepository, times(1)).save(argThat(droneModel ->
                droneModel.getManufacturer().equals("Parrot") &&
                        droneModel.getMaxSpeed() == 65.0 &&
                        droneModel.getRecommendedSpeed() == 55.0 &&
                        droneModel.getBatteryCapacity() == 5000 &&
                        droneModel.getMaxTripLength() == 350
        ));
    }
    @Test
    public void testUpdateDroneModelNotFound() {
        when(droneModelRepository.findById(anyLong())).thenReturn(Optional.empty());

        DroneModelDto updatedDto = new DroneModelDto(1L, "Parrot", 65.0, 55.0, 5000, 350);

        assertThrows(ResourceNotFoundException.class, () -> {
            droneModelService.updateDroneModel(999L, updatedDto);
        });

        verify(droneModelRepository, times(1)).findById(999L);
    }

    @Test
    public void testDeleteDroneModel() {
        when(droneModelRepository.findById(anyLong())).thenReturn(Optional.of(testDroneModel));
        doNothing().when(droneModelRepository).deleteById(anyLong());

        droneModelService.deleteDroneModel(1L);

        verify(droneModelRepository, times(1)).findById(1L);
        verify(droneModelRepository, times(1)).delete(any(DroneModel.class));
    }

    @Test
    public void testDeleteDroneModelNotFound() {
        when(droneModelRepository.findById(anyLong())).thenReturn(Optional.empty());

        assertThrows(ResourceNotFoundException.class, () -> {
            droneModelService.deleteDroneModel(999L);
        });

        verify(droneModelRepository, times(1)).findById(999L);
    }
}