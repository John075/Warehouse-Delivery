package com.warehouse_delivery.spring_boot.services;

import com.warehouse_delivery.spring_boot.dto.PackageDto;
import com.warehouse_delivery.spring_boot.entity.Package;
import com.warehouse_delivery.spring_boot.enums.PackageStatus;
import com.warehouse_delivery.spring_boot.mapper.PackageMapper;
import com.warehouse_delivery.spring_boot.messages.errors.ResourceNotFoundException;
import com.warehouse_delivery.spring_boot.repositories.PackageRepository;
import com.warehouse_delivery.spring_boot.services.impl.PackageServiceImpl;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyLong;
import static org.mockito.Mockito.*;

class PackageServiceTest {

    @Mock
    private PackageRepository packageRepository;

    @InjectMocks
    private PackageServiceImpl packageService;

    private Package testPackage;
    private PackageDto testPackageDto;

    @BeforeEach
    public void setUp() {
        MockitoAnnotations.openMocks(this);

        testPackage = new Package();
        testPackage.setId(1L);
        testPackage.setName("Sample Package");
        testPackage.setStatus(PackageStatus.IN_TRANSIT);

        testPackageDto = PackageMapper.mapToPackageDto(testPackage, true);
    }

    @Test
    public void testGetPackageSuccess() {
        when(packageRepository.findById(anyLong())).thenReturn(Optional.of(testPackage));

        PackageDto result = packageService.getPackage(1L);

        assertNotNull(result);
        assertEquals(1L, result.getId());
        assertEquals("Sample Package", result.getName());
        assertEquals(PackageStatus.IN_TRANSIT, result.getStatus());

        verify(packageRepository, times(1)).findById(1L);
    }

    @Test
    public void testGetPackageNotFound() {
        when(packageRepository.findById(anyLong())).thenReturn(Optional.empty());
        assertThrows(ResourceNotFoundException.class, () -> packageService.getPackage(1L));
        verify(packageRepository, times(1)).findById(1L);
    }

    @Test
    public void testGetAllPackages() {
        Package secondPackage = new Package();
        secondPackage.setId(2L);
        secondPackage.setName("Second Package");
        secondPackage.setStatus(PackageStatus.DELIVERED);

        List<Package> packageList = Arrays.asList(testPackage, secondPackage);
        when(packageRepository.findAll()).thenReturn(packageList);

        List<PackageDto> result = packageService.getAllPackages();

        assertNotNull(result);
        assertEquals(2, result.size());
        assertEquals(1L, result.get(0).getId());
        assertEquals(2L, result.get(1).getId());

        verify(packageRepository, times(1)).findAll();
    }

    @Test
    public void testRegisterPackage() {
        when(packageRepository.save(any(Package.class))).thenReturn(testPackage);

        PackageDto result = packageService.registerPackage(testPackageDto);

        assertNotNull(result);
        assertEquals(1L, result.getId());
        assertEquals("Sample Package", result.getName());

        verify(packageRepository, times(1)).save(any(Package.class));
    }

    @Test
    public void testUpdatePackageSuccess() {
        when(packageRepository.findById(anyLong())).thenReturn(Optional.of(testPackage));
        when(packageRepository.save(any(Package.class))).thenReturn(testPackage);

        PackageDto result = packageService.updatePackage(1L, testPackageDto);

        assertNotNull(result);
        assertEquals(1L, result.getId());
        assertEquals("Sample Package", result.getName());

        verify(packageRepository, times(1)).findById(1L);
        verify(packageRepository, times(1)).save(any(Package.class));
    }

    @Test
    public void testUpdatePackageNotFound() {
        when(packageRepository.findById(anyLong())).thenReturn(Optional.empty());
        assertThrows(ResourceNotFoundException.class, () -> packageService.updatePackage(1L, testPackageDto));
        verify(packageRepository, times(1)).findById(1L);
    }

    @Test
    public void testDeletePackageSuccess() {
        when(packageRepository.findById(anyLong())).thenReturn(Optional.of(testPackage));

        packageService.deletePackage(1L);

        verify(packageRepository, times(1)).findById(1L);
        verify(packageRepository, times(1)).delete(testPackage);
    }

    @Test
    public void testDeletePackageNotFound() {
        when(packageRepository.findById(anyLong())).thenReturn(Optional.empty());

        assertThrows(ResourceNotFoundException.class, () -> packageService.deletePackage(1L));

        verify(packageRepository, times(1)).findById(1L);
    }
}