import axios from 'axios';

const API_URL = "http://localhost:8080/api/drone-model";

class DroneModelService {
    getDroneModels() {
        return axios.get(API_URL);
    }

    registerDroneModel(drone_model) {
        return axios.post(API_URL, drone_model)
    }
}

const droneModelServiceInstance = new DroneModelService();
export default droneModelServiceInstance;