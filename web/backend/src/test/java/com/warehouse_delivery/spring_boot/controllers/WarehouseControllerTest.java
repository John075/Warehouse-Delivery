package com.warehouse_delivery.spring_boot.controllers;

import com.warehouse_delivery.spring_boot.dto.WarehouseDto;
import com.warehouse_delivery.spring_boot.services.WarehouseService;
import com.warehouse_delivery.spring_boot.entity.Address;
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

import java.util.List;

import static org.mockito.ArgumentMatchers.anyLong;
import static org.mockito.ArgumentMatchers.argThat;
import static org.mockito.Mockito.when;
import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.*;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

@SpringBootTest
@AutoConfigureMockMvc
public class WarehouseControllerTest {

    @Autowired
    private MockMvc mockMvc;

    @MockBean
    private WarehouseService warehouseService;

    @InjectMocks
    private WarehouseController warehouseController;

    private WarehouseDto testWarehouse;

    @BeforeEach
    public void setUp() {
        MockitoAnnotations.openMocks(this);
        this.mockMvc = MockMvcBuilders.standaloneSetup(warehouseController).build();

        // Set up the test WarehouseDto
        Address address = new Address(37.7749, -122.4194);
        testWarehouse = new WarehouseDto();
        testWarehouse.setId(1L);
        testWarehouse.setName("Main Warehouse");
        testWarehouse.setAddress(address);
        testWarehouse.setCapacity(500);
    }

    @Test
    public void testGetWarehouse() throws Exception {
        // Mock the service call
        when(warehouseService.getWarehouseById(anyLong())).thenReturn(testWarehouse);

        mockMvc.perform(get("/api/warehouse/1")
                        .accept(MediaType.APPLICATION_JSON))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.id").value(1L))
                .andExpect(jsonPath("$.name").value("Main Warehouse"))
                .andExpect(jsonPath("$.address.latitude").value(37.7749))
                .andExpect(jsonPath("$.address.longitude").value(-122.4194))
                .andExpect(jsonPath("$.capacity").value(500));
    }

    @Test
    public void testGetAllWarehouses() throws Exception {
        // Mock the service call
        when(warehouseService.getAllWarehouses()).thenReturn(List.of(testWarehouse));

        mockMvc.perform(get("/api/warehouse")
                        .accept(MediaType.APPLICATION_JSON))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.length()").value(1))
                .andExpect(jsonPath("$[0].id").value(1L))
                .andExpect(jsonPath("$[0].name").value("Main Warehouse"))
                .andExpect(jsonPath("$[0].address.latitude").value(37.7749))
                .andExpect(jsonPath("$[0].address.longitude").value(-122.4194))
                .andExpect(jsonPath("$[0].capacity").value(500));
    }

    @Test
    public void testRegisterWarehouse() throws Exception {
        // Mock service call
        when(warehouseService.createWarehouse(Mockito.any(WarehouseDto.class))).thenReturn(testWarehouse);

        String warehouseJson = TestUtils.asJsonString(testWarehouse);

        mockMvc.perform(post("/api/warehouse")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(warehouseJson))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.id").value(1L))
                .andExpect(jsonPath("$.name").value("Main Warehouse"))
                .andExpect(jsonPath("$.address.latitude").value(37.7749))
                .andExpect(jsonPath("$.address.longitude").value(-122.4194))
                .andExpect(jsonPath("$.capacity").value(500));
    }

    @Test
    public void testUpdateWarehouse() throws Exception {
        // Update the testWarehouse details
        testWarehouse.setName("Updated Warehouse");
        testWarehouse.setCapacity(600);

        // Mock service call
        when(warehouseService.updateWarehouse(anyLong(), argThat(warehouseDto ->
                warehouseDto.getId().equals(testWarehouse.getId()) && warehouseDto.getName().equals("Updated Warehouse"))))
                .thenReturn(testWarehouse);

        String warehouseJson = TestUtils.asJsonString(testWarehouse);

        mockMvc.perform(put("/api/warehouse/1")
                        .contentType(MediaType.APPLICATION_JSON)
                        .content(warehouseJson))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.id").value(1L))
                .andExpect(jsonPath("$.name").value("Updated Warehouse"))
                .andExpect(jsonPath("$.capacity").value(600));
    }

    @Test
    public void testDeleteWarehouse() throws Exception {
        // Mock the service call for deleting the warehouse
        mockMvc.perform(delete("/api/warehouse/1")
                        .accept(MediaType.APPLICATION_JSON))
                .andExpect(status().isNoContent());
    }
}