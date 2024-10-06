package com.warehouse_delivery.spring_boot.repositories;

import com.warehouse_delivery.spring_boot.entity.DroneModel;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface DroneModelRepository extends JpaRepository<DroneModel, Long> {
}
