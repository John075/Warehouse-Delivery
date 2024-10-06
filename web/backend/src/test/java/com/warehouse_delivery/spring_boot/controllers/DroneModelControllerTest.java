package com.warehouse_delivery.spring_boot.controllers;

import com.warehouse_delivery.spring_boot.dto.DroneModelDto;
import com.warehouse_delivery.spring_boot.services.DroneModelService;
import com.warehouse_delivery.spring_boot.utils.TestUtils;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.InjectMocks;
import org.mockito.Mockito;
import org.mockito.MockitoAnnotations;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.AutoConfigureMockMvc;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.boot.test.mock.mockito.MockBean;
import org.springframework.http.MediaType;
import org.springframework.test.web.servlet.MockMvc;
import org.springframework.test.web.servlet.setup.MockMvcBuilders;

import java.util.Arrays;
import java.util.List;

import static org.mockito.ArgumentMatchers.anyLong;
import static org.mockito.Mockito.*;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.*;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.*;

@SpringBootTest
@AutoConfigureMockMvc
public class DroneModelControllerTest {

    @Autowired
    private MockMvc mockMvc;

    @MockBean
    private DroneModelService droneModelService;

    @InjectMocks
    private DroneModelController droneModelController;

    private DroneModelDto testDroneModel;

    @BeforeEach
    public void setUp() {
        MockitoAnnotations.openMocks(this);
        this.mockMvc = MockMvcBuilders.standaloneSetup(droneModelController).build();

        testDroneModel = new DroneModelDto(1L, "DJI", 60.5, 50.0, 4000, 300);

        // Mocking service calls
        when(droneModelService.getDroneModelById(1L)).thenReturn(testDroneModel);
        when(droneModelService.createDroneModel(any(DroneModelDto.class))).thenReturn(testDroneModel);
        when(droneModelService.updateDroneModel(anyLong(), any(DroneModelDto.class))).thenReturn(testDroneModel);
    }

    @Test
    public void testGetAllDroneModels() throws Exception {
        List<DroneModelDto> droneModels = Arrays.asList(testDroneModel);
        when(droneModelService.getAllDroneModels()).thenReturn(droneModels);

        mockMvc.perform(get("/api/drone-model").accept(MediaType.APPLICATION_JSON))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.length()").value(1))
                .andExpect(jsonPath("$[0].id").value(1L))
                .andExpect(jsonPath("$[0].manufacturer").value("DJI"))
                .andExpect(jsonPath("$[0].maxSpeed").value(60.5))
                .andExpect(jsonPath("$[0].recommendedSpeed").value(50.0))
                .andExpect(jsonPath("$[0].batteryCapacity").value(4000))
                .andExpect(jsonPath("$[0].maxTripLength").value(300));
    }

    @Test
    public void testGetDroneModelById() throws Exception {
        mockMvc.perform(get("/api/drone-model/1").accept(MediaType.APPLICATION_JSON))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.id").value(1L))
                .andExpect(jsonPath("$.manufacturer").value("DJI"))
                .andExpect(jsonPath("$.maxSpeed").value(60.5))
                .andExpect(jsonPath("$.recommendedSpeed").value(50.0))
                .andExpect(jsonPath("$.batteryCapacity").value(4000))
                .andExpect(jsonPath("$.maxTripLength").value(300));
    }

    @Test
    public void testCreateDroneModel() throws Exception {
        String droneModelJson = TestUtils.asJsonString(testDroneModel);

        mockMvc.perform(post("/api/drone-model")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(droneModelJson))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.id").value(1L))
                .andExpect(jsonPath("$.manufacturer").value("DJI"))
                .andExpect(jsonPath("$.maxSpeed").value(60.5))
                .andExpect(jsonPath("$.recommendedSpeed").value(50.0))
                .andExpect(jsonPath("$.batteryCapacity").value(4000))
                .andExpect(jsonPath("$.maxTripLength").value(300));
    }

    @Test
    public void testUpdateDroneModel() throws Exception {
        testDroneModel.setManufacturer("Parrot");
        testDroneModel.setMaxSpeed(65.0);

        String droneModelJson = TestUtils.asJsonString(testDroneModel);

        mockMvc.perform(put("/api/drone-model/1")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(droneModelJson))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.id").value(1L))
                .andExpect(jsonPath("$.manufacturer").value("Parrot"))
                .andExpect(jsonPath("$.maxSpeed").value(65.0))
                .andExpect(jsonPath("$.recommendedSpeed").value(50.0))
                .andExpect(jsonPath("$.batteryCapacity").value(4000))
                .andExpect(jsonPath("$.maxTripLength").value(300));
    }

    @Test
    public void testDeleteDroneModel() throws Exception {
        doNothing().when(droneModelService).deleteDroneModel(1L);

        mockMvc.perform(delete("/api/drone-model/1"))
                .andExpect(status().isNoContent());
    }
}