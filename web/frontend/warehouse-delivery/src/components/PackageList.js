import React, { useState, useEffect } from 'react';
import PackageService from '../services/PackageService';
import { Container, Table, Button, Spinner, Alert, Modal, Form } from 'react-bootstrap';
import { FaPlus } from 'react-icons/fa';
import DroneService from '../services/DroneService';
import WarehouseService from "../services/WarehouseService";  // Assuming you have this service

function PackageList() {
    const [packages, setPackages] = useState([]);
    const [drones, setDrones] = useState([]);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState(null);
    const [showModal, setShowModal] = useState(false);
    const [packageName, setPackageName] = useState('');
    const [assignedDrone, setAssignedDrone] = useState('');
    const [warehouses, setWarehouses] = useState([]);
    const [selectedWarehouse, setSelectedWarehouse] = useState('');
    const [selectedPackage, setSelectedPackage] = useState(null);
    const [status, setStatus] = useState('');
    const [destination, setDestination] = useState({ street: '', city: '', state: '', zipCode: '', country: '' });
    const [priority, setPriority] = useState(0);
    const [successMessage, setSuccessMessage] = useState('');


    useEffect(() => {
        PackageService.getPackages().then((response) => {
            setPackages(response.data);
            setLoading(false);
        }).catch(error => {
            console.error("Error fetching packages:", error);
            setError("There was an error fetching the package data.");
            setLoading(false);
        });

        DroneService.getDrones()
            .then((response) => {
                setDrones(response.data);
            })
            .catch((error) => {
                console.error("Error fetching drones:", error);
            });

        WarehouseService.getWarehouses()
            .then((response) => {
                setWarehouses(response.data);
            })
            .catch((error) => {
                console.error("Error fetching warehouses:", error);
            });
    }, []);

    const handleRegisterClick = () => {
        setShowModal(true);
    };

    const handleEditClick = (pkg) => {
        setPackageName(pkg.name);
        setAssignedDrone(pkg.assignedDrone || '');
        setSelectedWarehouse(pkg.warehouse || '');
        setStatus(pkg.status);
        setDestination(pkg.destination || { latitude: '', longitude: '' });
        setPriority(pkg.priority);
        setSelectedPackage(pkg);  // Mark the package being edited
        setShowModal(true);
    };

    const handleCloseModal = () => {
        setShowModal(false);
        setPackageName('');
        setAssignedDrone('');
        setStatus('');
        setDestination({ latitude: '', longitude: '' });
        setPriority(0);
        setSelectedWarehouse('');
        setSelectedPackage(null);  // Reset selected package for the next create action
        setSuccessMessage('');
    };

    const handleRegisterPackage = (e) => {
        e.preventDefault();
        const updatedPackage = {
            id: selectedPackage ? selectedPackage.id : undefined,
            name: packageName,
            assignedDrone: assignedDrone || null,
            status,
            destination,
            priority,
            warehouse: selectedWarehouse || null,
        };

        if (selectedPackage) {
            PackageService.updatePackage(selectedPackage.id, updatedPackage)
                .then(() => {
                    setPackages(packages.map(pkg =>
                        pkg.id === selectedPackage.id ? { ...updatedPackage, id: selectedPackage.id } : pkg
                    ));
                    setSuccessMessage('Package successfully updated!');
                    setTimeout(handleCloseModal, 1500);
                })
                .catch((err) => {
                    setError('Error updating package: ' + err.message);
                });
        } else {
            PackageService.registerPackage(updatedPackage)
                .then(() => {
                    setSuccessMessage('Package successfully registered!');
                    setPackages([...packages, { ...updatedPackage, id: packages.length + 1 }]);
                    setTimeout(handleCloseModal, 1500);
                })
                .catch((err) => {
                    setError('Error registering package: ' + err.message);
                });
        }
    };

    return (
        <Container className='mt-4'>
            <h2 className="text-center">Packages List</h2>

            {error && (
                <Alert variant="danger" className="mt-4">
                    {error}
                </Alert>
            )}

            {successMessage && (
                <Alert variant="success" className="mt-4">
                    {successMessage}
                </Alert>
            )}

            {loading && (
                <div className="text-center mt-5">
                    <Spinner animation="border" role="status">
                        <span className="visually-hidden">Loading...</span>
                    </Spinner>
                </div>
            )}

            <div className="d-flex justify-content-center mb-4">
                <Button
                    variant="primary"
                    className="register-btn"
                    size="lg"
                    onClick={handleRegisterClick}
                >
                    <FaPlus /> Register New Package
                </Button>
            </div>

            <Table striped bordered hover className="mt-4">
                <thead>
                <tr>
                    <th>ID</th>
                    <th>Package Name</th>
                    <th>Assigned Drone</th>
                    <th>Warehouse</th>
                    <th>Status</th>
                    <th>Destination</th>
                    <th>Priority</th>
                    <th>Actions</th>
                </tr>
                </thead>
                <tbody>
                {packages.map(pkg => (
                    <tr key={pkg.id}>
                        <td>{pkg.id}</td>
                        <td>{pkg.name}</td>
                        <td>{pkg.assignedDrone ? pkg.assignedDrone.name : 'Unassigned'}</td>
                        <td>{pkg.warehouse ? pkg.warehouse.name : 'Not assigned'}</td>
                        <td>{pkg.status}</td>
                        <td>{pkg.destination ? `${pkg.destination.latitude}, ${pkg.destination.longitude}` : 'N/A'}</td>
                        <td>{pkg.priority}</td>
                        <td>
                            <Button
                                variant="success" type="submit"
                                onClick={() => handleEditClick(pkg)}
                            >
                                Edit
                            </Button>
                        </td>
                    </tr>
                ))}
                </tbody>
            </Table>

            <Modal show={showModal} onHide={handleCloseModal} centered>
                <Modal.Header closeButton>
                    <Modal.Title>{selectedPackage ? "Edit Package" : "Register a New Package"}</Modal.Title>
                </Modal.Header>
                <Modal.Body>
                    <Form onSubmit={handleRegisterPackage}>
                        <Form.Group controlId="packageName">
                            <Form.Label>Package Name</Form.Label>
                            <Form.Control
                                type="text"
                                placeholder="Enter package name"
                                value={packageName}
                                onChange={(e) => setPackageName(e.target.value)}
                                required
                            />
                        </Form.Group>

                        <Form.Group controlId="assignedDrone">
                            <Form.Label>Assigned Drone (Optional)</Form.Label>
                            <Form.Control
                                as="select"
                                value={assignedDrone ? assignedDrone.id : ''}
                                onChange={(e) => {
                                    const selectedDrone = drones.find(drone => drone.id === parseInt(e.target.value));
                                    setAssignedDrone(selectedDrone || null); // Update assignedDrone with the full object
                                }}
                            >
                                <option value="">Select Drone</option>
                                {drones.map(drone => (
                                    <option key={drone.id} value={drone.id}>
                                        {drone.name}
                                    </option>
                                ))}
                            </Form.Control>
                        </Form.Group>

                        <Form.Group controlId="warehouse">
                            <Form.Label>Select Warehouse</Form.Label>
                            <Form.Control
                                as="select"
                                value={selectedWarehouse ? selectedWarehouse.id : ''}
                                onChange={(e) => {
                                    const selected = warehouses.find(warehouse => warehouse.id === parseInt(e.target.value));
                                    setSelectedWarehouse(selected || null);  // Store full warehouse object
                                }}
                            >
                                <option value="">Select Warehouse</option>
                                {warehouses.map(warehouse => (
                                    <option key={warehouse.id} value={warehouse.id}>
                                        {warehouse.name}
                                    </option>
                                ))}
                            </Form.Control>
                        </Form.Group>

                        <Form.Group controlId="status">
                            <Form.Label>Status</Form.Label>
                            <Form.Control
                                as="select"
                                value={status}
                                onChange={(e) => setStatus(e.target.value)}
                            >
                                <option value="">Select Status</option>
                                <option value="IN_TRANSIT">In Transit</option>
                                <option value="DELIVERED">Delivered</option>
                                <option value="PENDING">Pending</option>
                            </Form.Control>
                        </Form.Group>

                        <Form.Group controlId="destination">
                            <Form.Label>Destination</Form.Label>
                            <Form.Control
                                type="number"
                                placeholder="Latitude"
                                value={destination.latitude}
                                onChange={(e) => setDestination({ ...destination, latitude: e.target.value })}
                                required
                            />
                            <Form.Control
                                type="number"
                                placeholder="Longitude"
                                value={destination.longitude}
                                onChange={(e) => setDestination({ ...destination, longitude: e.target.value })}
                                className="mt-2"
                                required
                            />
                        </Form.Group>

                        <Form.Group controlId="priority">
                            <Form.Label>Priority</Form.Label>
                            <Form.Control
                                type="number"
                                value={priority}
                                onChange={(e) => setPriority(e.target.value)}
                            />
                        </Form.Group>

                        <Button variant="success" type="submit" className="mt-3 w-100">
                            {selectedPackage ? "Update Package" : "Register Package"}
                        </Button>
                    </Form>
                </Modal.Body>
            </Modal>
        </Container>
    );
}

export default PackageList;