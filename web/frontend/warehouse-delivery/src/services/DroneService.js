import axios from 'axios';

const API_URL = "http://localhost:8080/api/drone";

class DroneService {
    getDrones() {
        return axios.get(API_URL);
    }

    registerDrone(drone) {
        return axios.post(API_URL, drone)
    }
}

const droneServiceInstance = new DroneService();
export default droneServiceInstance;