package com.warehouse_delivery.spring_boot.broadcaster;

import com.warehouse_delivery.spring_boot.entity.Drone;
import com.warehouse_delivery.spring_boot.mapper.DroneMapper;
import jakarta.persistence.PostPersist;
import jakarta.persistence.PostRemove;
import jakarta.persistence.PostUpdate;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Component;

@Component
public class DroneEntityListener {

    private static DroneUpdateBroadcaster broadcaster;

    @Autowired
    public void init(DroneUpdateBroadcaster broadcaster) {
        DroneEntityListener.broadcaster = broadcaster;
    }

    @PostPersist
    @PostUpdate
    @PostRemove
    public void afterAnyUpdate(Drone drone) {
        broadcaster.broadcastUpdate(drone.getId(), "drone-general", DroneMapper.mapToDroneDto(drone, true));
    }
}
