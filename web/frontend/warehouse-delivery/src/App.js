import React from 'react';
import './App.css';
import 'bootstrap/dist/css/bootstrap.min.css';
import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';
import DroneList from "./components/DroneList";
import PackageList from "./components/PackageList";
import HeaderComponent from "./components/HeaderComponent";
import WelcomePage from "./components/WelcomePage";
import DroneModelList from "./components/DroneModelList";
import WarehouseList from "./components/WarehouseList";

function App() {
    return (
        <Router>
            <div className="App">
                <HeaderComponent/>
                <Routes>
                    <Route path="/" element={<WelcomePage />} />
                    <Route path="/drones" element={<DroneList />} />
                    <Route path="/packages" element={<PackageList />} />
                    <Route path="/drone-models" element={<DroneModelList />} />
                    <Route path="/warehouses" element={<WarehouseList />} />
                </Routes>
            </div>
        </Router>
    );
}

export default App;