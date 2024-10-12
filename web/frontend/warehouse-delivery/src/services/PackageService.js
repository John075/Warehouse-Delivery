import axios from 'axios';

const API_URL = "http://localhost:8080/api/package";

class PackageService {
    getPackages() {
        return axios.get(API_URL);
    }

    registerPackage(packageEntity) {
        return axios.post(API_URL, packageEntity);
    }

    updatePackage(id, updatedPackage) {
        return axios.put(API_URL + "/" + id, updatedPackage);
    }
}

const packageServiceInstance = new PackageService();
export default packageServiceInstance;