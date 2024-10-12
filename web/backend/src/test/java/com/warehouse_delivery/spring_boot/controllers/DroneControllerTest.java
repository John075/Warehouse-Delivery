package com.warehouse_delivery.spring_boot.controllers;

import com.warehouse_delivery.spring_boot.dto.DroneDto;
import com.warehouse_delivery.spring_boot.enums.DroneStatus;
import com.warehouse_delivery.spring_boot.services.DroneService;
import com.warehouse_delivery.spring_boot.utils.TestUtils;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;
import org.springframework.boot.test.autoconfigure.web.servlet.AutoConfigureMockMvc;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.http.MediaType;
import org.springframework.test.web.servlet.MockMvc;
import org.springframework.test.web.servlet.setup.MockMvcBuilders;

import java.time.LocalDateTime;
import java.util.Arrays;
import java.util.Collections;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.*;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.*;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

@SpringBootTest
@AutoConfigureMockMvc
class DroneControllerTest {

    @Mock
    private DroneService droneService;
    @InjectMocks
    private DroneController droneController;
    private MockMvc mockMvc;
    private DroneDto testDroneOne;

    @BeforeEach
    void setUp() {
        MockitoAnnotations.openMocks(this);
        mockMvc = MockMvcBuilders.standaloneSetup(droneController).build();

        testDroneOne = new DroneDto();
        testDroneOne.setId(1L);
        testDroneOne.setName("Delivery Drone A");
        testDroneOne.setStatus(DroneStatus.IN_TRANSIT);
        testDroneOne.setLastLatitudeLocation(38.89511);
        testDroneOne.setLastLongitudeLocation(-77.03637);
        testDroneOne.setBatteryLife(85.5);
        testDroneOne.setLastMaintenanceDate(LocalDateTime.now().minusDays(10));
        testDroneOne.setConnectedToSystem(true);
        testDroneOne.setPackages(Collections.emptyList());
    }

    @Test
    void testGetDrone() throws Exception {
        when(droneService.getDrone(1L)).thenReturn(testDroneOne);

        mockMvc.perform(get("/api/drone/1")
                        .accept(MediaType.APPLICATION_JSON))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.id").value(1L))
                .andExpect(jsonPath("$.name").value("Delivery Drone A"))
                .andExpect(jsonPath("$.status").value("IN_TRANSIT"))
                .andExpect(jsonPath("$.lastLatitudeLocation").value(38.89511))
                .andExpect(jsonPath("$.lastLongitudeLocation").value(-77.03637))
                .andExpect(jsonPath("$.batteryLife").value(85.5))
                .andExpect(jsonPath("$.connectedToSystem").value(true))
                .andExpect(jsonPath("$.lastMaintenanceDate").exists());

        verify(droneService, times(1)).getDrone(1L);
    }

    @Test
    void testGetAllDrones() throws Exception {
        when(droneService.getAllDrones()).thenReturn(Arrays.asList(testDroneOne));

        mockMvc.perform(get("/api/drone")
                        .accept(MediaType.APPLICATION_JSON))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.length()").value(1))
                .andExpect(jsonPath("$[0].id").value(1L))
                .andExpect(jsonPath("$[0].name").value("Delivery Drone A"))
                .andExpect(jsonPath("$[0].status").value("IN_TRANSIT"))
                .andExpect(jsonPath("$[0].lastLatitudeLocation").value(38.89511))
                .andExpect(jsonPath("$[0].lastLongitudeLocation").value(-77.03637))
                .andExpect(jsonPath("$[0].batteryLife").value(85.5))
                .andExpect(jsonPath("$[0].connectedToSystem").value(true));

        verify(droneService, times(1)).getAllDrones();
    }

    @Test
    void testRegisterDrone() throws Exception {
        when(droneService.registerDrone(any(DroneDto.class))).thenReturn(testDroneOne);

        DroneDto newDrone = new DroneDto();
        newDrone.setName("New Drone");
        newDrone.setStatus(DroneStatus.IDLE);
        newDrone.setLastLatitudeLocation(40.7128);
        newDrone.setLastLongitudeLocation(-74.0060);
        newDrone.setBatteryLife(100);
        newDrone.setConnectedToSystem(true);

        mockMvc.perform(post("/api/drone")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(TestUtils.asJsonString(newDrone)))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.id").value(1L))
                .andExpect(jsonPath("$.name").value("Delivery Drone A"))
                .andExpect(jsonPath("$.status").value("IN_TRANSIT"))
                .andExpect(jsonPath("$.lastLatitudeLocation").value(38.89511))
                .andExpect(jsonPath("$.lastLongitudeLocation").value(-77.03637))
                .andExpect(jsonPath("$.batteryLife").value(85.5))
                .andExpect(jsonPath("$.connectedToSystem").value(true));

        verify(droneService, times(1)).registerDrone(any(DroneDto.class));
    }

    @Test
    void testUpdateDrone() throws Exception {
        DroneDto updatedDrone = new DroneDto();
        updatedDrone.setId(1L);
        updatedDrone.setName("Updated Drone");
        updatedDrone.setStatus(DroneStatus.IDLE);
        updatedDrone.setLastLatitudeLocation(37.7749);
        updatedDrone.setLastLongitudeLocation(-122.4194);
        updatedDrone.setBatteryLife(90);
        updatedDrone.setConnectedToSystem(false);

        when(droneService.updateDrone(eq(1L), any(DroneDto.class))).thenReturn(updatedDrone);

        mockMvc.perform(put("/api/drone/1")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(TestUtils.asJsonString(updatedDrone)))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.id").value(1L))
                .andExpect(jsonPath("$.name").value("Updated Drone"))
                .andExpect(jsonPath("$.status").value("IDLE"))
                .andExpect(jsonPath("$.lastLatitudeLocation").value(37.7749))
                .andExpect(jsonPath("$.lastLongitudeLocation").value(-122.4194))
                .andExpect(jsonPath("$.batteryLife").value(90))
                .andExpect(jsonPath("$.connectedToSystem").value(false));

        verify(droneService, times(1)).updateDrone(eq(1L), any(DroneDto.class));
    }

    @Test
    void testDeleteDrone() throws Exception {
        doNothing().when(droneService).deleteDrone(1L);

        mockMvc.perform(delete("/api/drone/1"))
                .andExpect(status().isNoContent());

        verify(droneService, times(1)).deleteDrone(1L);
    }

}