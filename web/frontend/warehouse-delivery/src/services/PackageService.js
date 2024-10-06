import axios from 'axios';

const API_URL = "http://localhost:8080/api/package";

class PackageService {
    getPackages() {
        return axios.get(API_URL);
    }

    registerPackage(packageEntity) {
        return axios.post(API_URL, packageEntity);
    }
}

const packageServiceInstance = new PackageService();
export default packageServiceInstance;