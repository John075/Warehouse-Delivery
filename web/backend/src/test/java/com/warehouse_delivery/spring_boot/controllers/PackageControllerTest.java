package com.warehouse_delivery.spring_boot.controllers;

import com.warehouse_delivery.spring_boot.dto.DroneDto;
import com.warehouse_delivery.spring_boot.dto.PackageDto;
import com.warehouse_delivery.spring_boot.dto.WarehouseDto;
import com.warehouse_delivery.spring_boot.entity.Address;
import com.warehouse_delivery.spring_boot.enums.PackageStatus;
import com.warehouse_delivery.spring_boot.services.PackageService;
import com.warehouse_delivery.spring_boot.utils.TestUtils;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.InjectMocks;
import org.mockito.MockitoAnnotations;
import org.springframework.boot.test.autoconfigure.web.servlet.AutoConfigureMockMvc;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.boot.test.mock.mockito.MockBean;
import org.springframework.http.MediaType;
import org.springframework.test.web.servlet.MockMvc;
import org.springframework.test.web.servlet.setup.MockMvcBuilders;

import java.util.Arrays;

import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.when;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.*;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

@SpringBootTest
@AutoConfigureMockMvc
public class PackageControllerTest {

    @MockBean
    private PackageService packageService;
    @InjectMocks
    private PackageController packageController;

    private MockMvc mockMvc;

    private PackageDto testPackage;
    private DroneDto testDrone;
    private WarehouseDto testWarehouse;

    @BeforeEach
    public void setUp() {
        MockitoAnnotations.openMocks(this);
        this.mockMvc = MockMvcBuilders.standaloneSetup(packageController).build();

        testDrone = new DroneDto();
        testDrone.setId(1L);
        testDrone.setName("Delivery Drone A");
        testDrone.setBatteryLife(80.5);
        testDrone.setLastLatitudeLocation(37.7749);
        testDrone.setLastLongitudeLocation(-122.4194);
        testDrone.setConnectedToSystem(true);

        testWarehouse = new WarehouseDto();
        testWarehouse.setId(1L);
        testWarehouse.setName("Main Warehouse");
        testWarehouse.setLatitude(40.7128);
        testWarehouse.setLongitude(-74.0060);
        testWarehouse.setCapacity(100);

        testPackage = new PackageDto();
        testPackage.setId(1L);
        testPackage.setName("Sample Package");
        testPackage.setStatus(PackageStatus.PENDING);
        testPackage.setPriority(1);
        testPackage.setOrderTime(System.currentTimeMillis());
        testPackage.setWarehouse(testWarehouse); // Assign the test warehouse
        testPackage.setAssignedDrone(testDrone); // Assign the test drone
        testPackage.setDestination(new Address("123 Street", "City", "State", "12345", "Country"));
    }

    @Test
    public void testGetPackage() throws Exception {
        when(packageService.getPackage(anyLong())).thenReturn(testPackage);

        mockMvc.perform(get("/api/package/1").accept(MediaType.APPLICATION_JSON))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.id").value(1L))
                .andExpect(jsonPath("$.name").value("Sample Package"))
                .andExpect(jsonPath("$.status").value("PENDING"))
                .andExpect(jsonPath("$.priority").value(1))
                .andExpect(jsonPath("$.assignedDrone.name").value("Delivery Drone A"))
                .andExpect(jsonPath("$.warehouse.name").value("Main Warehouse"))
                .andExpect(jsonPath("$.destination.street").value("123 Street"));
    }

    @Test
    public void testGetAllPackages() throws Exception {
        when(packageService.getAllPackages()).thenReturn(Arrays.asList(testPackage));

        mockMvc.perform(get("/api/package").accept(MediaType.APPLICATION_JSON))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.length()").value(1))
                .andExpect(jsonPath("$[0].id").value(1L))
                .andExpect(jsonPath("$[0].name").value("Sample Package"))
                .andExpect(jsonPath("$[0].assignedDrone.name").value("Delivery Drone A"))
                .andExpect(jsonPath("$[0].warehouse.name").value("Main Warehouse"));
    }

    @Test
    public void testRegisterPackage() throws Exception {
        when(packageService.registerPackage(any(PackageDto.class))).thenReturn(testPackage);

        String packageJson = TestUtils.asJsonString(testPackage);
        System.out.println(packageJson);

        mockMvc.perform(post("/api/package")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(packageJson))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.id").value(1L))
                .andExpect(jsonPath("$.name").value("Sample Package"))
                .andExpect(jsonPath("$.assignedDrone.name").value("Delivery Drone A"))
                .andExpect(jsonPath("$.warehouse.name").value("Main Warehouse"));
    }

    @Test
    public void testUpdatePackage() throws Exception {
        testPackage.setName("Updated Package");
        testPackage.setStatus(PackageStatus.DELIVERED);

        when(packageService.updatePackage(anyLong(), argThat(packageDto ->
                packageDto.getId().equals(testPackage.getId()) && packageDto.getName().equals("Updated Package"))))
                .thenReturn(testPackage);

        String packageJson = TestUtils.asJsonString(testPackage);

        mockMvc.perform(put("/api/package/1")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(packageJson))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.id").value(1L))
                .andExpect(jsonPath("$.name").value("Updated Package"))
                .andExpect(jsonPath("$.status").value("DELIVERED"))
                .andExpect(jsonPath("$.assignedDrone.name").value("Delivery Drone A"))
                .andExpect(jsonPath("$.warehouse.name").value("Main Warehouse"));
    }

    @Test
    public void testDeletePackage() throws Exception {
        mockMvc.perform(delete("/api/package/1"))
                .andExpect(status().isNoContent());
    }
}