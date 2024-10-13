package com.warehouse_delivery.spring_boot.broadcaster;

import com.warehouse_delivery.spring_boot.dto.DroneDto;
import org.springframework.stereotype.Component;
import org.springframework.web.servlet.mvc.method.annotation.SseEmitter;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArrayList;

@Component
public class DroneUpdateBroadcaster {

    private final Map<Long, List<SseEmitter>> emitterMap = new ConcurrentHashMap<>();
    public SseEmitter registerEmitter(long droneId, DroneDto dto) {
        SseEmitter emitter = new SseEmitter(0L);
        emitterMap.computeIfAbsent(droneId, id -> new CopyOnWriteArrayList<>()).add(emitter);

        try {
            emitter.send(SseEmitter.event().name("message").data("Connection has been established."));
            emitter.send(SseEmitter.event().name("drone-initial").data(dto));
        } catch (IOException e) {
            emitterMap.get(droneId).remove(emitter);
            return null;
        }

        emitter.onCompletion(() -> emitterMap.get(droneId).remove(emitter));
        emitter.onTimeout(() -> emitterMap.get(droneId).remove(emitter));

        return emitter;
    }

    public void broadcastUpdate(long droneId, String type, Object state) {
        List<SseEmitter> emitters = emitterMap.get(droneId);
        System.out.println("Sending event to " + (emitters == null ? "0" : emitters.size()) + " emitter(s).");

        if (emitters == null) {
            return; // No emitters registered for this drone ID
        }

        List<SseEmitter> emittersNotListening = new ArrayList<>();
        emitters.forEach(emitter -> {
            try {
                emitter.send(SseEmitter.event().name(type).data(state));
            } catch (IOException e) {
                emittersNotListening.add(emitter);
            }
        });

        emitters.removeAll(emittersNotListening);
        if (emitters.isEmpty()) {
            emitterMap.remove(droneId);
        }
    }
}
