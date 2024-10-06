package com.warehouse_delivery.spring_boot.controllers;

import com.warehouse_delivery.spring_boot.dto.PackageDto;
import com.warehouse_delivery.spring_boot.repositories.PackageRepository;
import com.warehouse_delivery.spring_boot.services.PackageService;
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
 * Test class for testing the Package REST API.
 * It contains unit tests for retrieving package details and all packages.
 */
@SpringBootTest
@AutoConfigureMockMvc
public class PackageControllerTest {

    @Autowired
    private PackageRepository packageRepository;

    @Autowired
    private PackageService packageService;

    @Autowired
    private EntityManager entityManager;

    @Autowired
    private MockMvc mvc;

    private PackageDto testPackageOne;

    /**
     * Set up method to run before each test case.
     * Deletes all packages from the repository, resets the sequence of the package ID to 1,
     * and creates a test package to be used for tests.
     */
    @BeforeEach
    public void setUp() {
        // Delete all packages currently in test repo to check native ID sequencing and be consistent
        packageRepository.deleteAll();
        entityManager.createNativeQuery("ALTER SEQUENCE package_id_seq RESTART WITH 1").executeUpdate();

        // Create a test package for us to check requirements against
        PackageDto testPackageOne = new PackageDto();
        testPackageOne.setName("Flowers from Amazon");
        testPackageOne.setId(1L);
        this.testPackageOne = packageService.registerPackage(testPackageOne);
    }

    /**
     * Test case for testing the GET request to retrieve a specific package by ID.
     * Verifies that the returned package has the correct ID and name.
     *
     * @throws Exception if any error occurs during the request
     */
    @Test
    @Transactional
    public void testGetPackage() throws Exception {
        mvc.perform(get("/api/package/" + testPackageOne.getId()).accept(MediaType.APPLICATION_JSON))
                .andExpect(status().isOk())
                .andExpect(jsonPath("$.id").value(1L)) // Check the id of the first entry
                .andExpect(jsonPath("$.name").value("Flowers from Amazon")); //Check name of the first entry
    }

    /**
     * Test case for testing the GET request to retrieve all packages.
     * Verifies that the length of the response array is 1 and that the first package has the correct ID and name.
     *
     * @throws Exception if any error occurs during the request
     */
    @Test
    @Transactional
    public void testGetAllPackages() throws Exception {
        mvc.perform(get("/api/package").accept(MediaType.APPLICATION_JSON))
                .andExpect(status().isOk()) .andExpect(jsonPath("$.length()").value(1))
                .andExpect(jsonPath("$[0].id").value(1L))  // Check the id of the first entry
                .andExpect(jsonPath("$[0].name").value("Flowers from Amazon"));  // Check the name of the first entry
    }


}
