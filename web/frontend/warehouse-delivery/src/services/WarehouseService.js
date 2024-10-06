import axios from 'axios';

const API_URL = "http://localhost:8080/api/warehouse";

class WarehouseService {
    getWarehouses() {
        return axios.get(API_URL);
    }

    registerWarehouse(warehouse) {
        return axios.post(API_URL, warehouse)
    }
}

const warehouseServiceInstance = new WarehouseService();
export default warehouseServiceInstance;