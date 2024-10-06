package com.warehouse_delivery.spring_boot.repositories;

import com.warehouse_delivery.spring_boot.entity.Package;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface PackageRepository extends JpaRepository<Package, Long> {
}
