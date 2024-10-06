package com.warehouse_delivery.spring_boot.controllers;

import com.warehouse_delivery.spring_boot.dto.DroneDto;
import com.warehouse_delivery.spring_boot.repositories.DroneRepository;
import com.warehouse_delivery.spring_boot.services.DroneService;
import jakarta.persistence.EntityManager;
import jakarta.transaction.Transactional;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.autoconfigure.web.servlet.AutoConfigureMockMvc;
import org.springframework.boot.test.context.SpringBootTest;
import org.springframework.http.MediaType;
import org.springframework.test.web.servlet.MockMvc;

import static org.springframework.test.web.servlet.request.MockMvcRequestBuilders.get;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.jsonPath;
import static org.springframework.test.web.servlet.result.MockMvcResultMatchers.status;

/**
 * Test class for testing the Drone REST API.
 * It contains unit tests for retrieving drone details and all drones.
 */
@SpringBootTest
@AutoConfigureMockMvc
public class DroneControllerTest {

    @Autowired
    private DroneRepository droneRepository;

    @Autowired
    private DroneService droneService;

    @Autowired
    private EntityManager entityManager;

    @Autowired
    private MockMvc mvc;

    private DroneDto testDroneOne;

    /**
     * Set up method to run before each test case.
     * Deletes all drones from the repository, resets the sequence of the drone ID to 1,
     * and creates a test drone to be used for tests.
     */
    @BeforeEach
    public void setUp() {
        // Delete all drones currently in test repo to check native ID sequencing and be consistent
        droneRepository.deleteAll();
        entityManager.createNativeQuery("ALTER SEQUENCE drone_id_seq RESTART WITH 1").executeUpdate();

        // Create a test drone for us to check requirements against
        DroneDto testDroneOne = new DroneDto();
        testDroneOne.setName("Delivery Drone A");
        testDroneOne.setId(1L);  // Use 1L for consistent testing
        this.testDroneOne = droneService.registerDrone(testDroneOne);
    }

    /**
     * Test case for testing the GET request to retrieve a specific drone by ID.
     * Verifies that the returned drone has the correct ID and name.
     *
     * @throws Exception if any error occurs during the request
     */
    @Test
    @Transactional
    public void testGetDrone() throws Exception {
        mvc.perform(get("/api/drone/" + testDroneOne.getId()).accept(MediaType.APPLICATION_JSON))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.id").value(1L))
                .andExpect(jsonPath("$.name").value("Delivery Drone A"));
    }

    /**
     * Test case for testing the GET request to retrieve all drones.
     * Verifies that the length of the response array is 1 and that the first drone has the correct ID and name.
     *
     * @throws Exception if any error occurs during the request
     */
    @Test
    @Transactional
    public void testGetAllDrones() throws Exception {
        mvc.perform(get("/api/drone").accept(MediaType.APPLICATION_JSON))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.length()").value(1))
                .andExpect(jsonPath("$[0].id").value(1L))
                .andExpect(jsonPath("$[0].name").value("Delivery Drone A"));
    }
}