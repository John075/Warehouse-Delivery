import React, { useState, useEffect } from 'react';
import DroneService from '../services/DroneService';
import DroneModelService from '../services/DroneModelService';  // Assuming you have a service to fetch drone models
import WarehouseService from '../services/WarehouseService';  // Assuming you have a service to fetch warehouses
import { Container, Card, Row, Col, Button, Spinner, Alert, Modal, Form } from 'react-bootstrap';
import { FaPlus } from 'react-icons/fa';

function DroneList() {
    const [drones, setDrones] = useState([]);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState(null);
    const [showModal, setShowModal] = useState(false);
    const [droneName, setDroneName] = useState('');
    const [status, setStatus] = useState('IN_TRANSIT');
    const [batteryLife, setBatteryLife] = useState(100);
    const [warehouseId, setWarehouseId] = useState('');
    const [droneModelId, setDroneModelId] = useState('');
    const [warehouses, setWarehouses] = useState([]);
    const [droneModels, setDroneModels] = useState([]);
    const [successMessage, setSuccessMessage] = useState('');

    useEffect(() => {
        DroneService.getDrones().then((response) => {
            setDrones(response.data);
            setLoading(false);
        }).catch(error => {
            console.error("Error fetching drones:", error);
            setError("There was an error fetching the drone data.");
            setLoading(false);
        });

        // Fetch available warehouses and drone models
        WarehouseService.getWarehouses().then((response) => {
            setWarehouses(response.data);
        });

        DroneModelService.getDroneModels().then((response) => {
            setDroneModels(response.data);
        });
    }, []);

    const handleRegisterClick = () => {
        setShowModal(true);
    };

    const handleCloseModal = () => {
        setShowModal(false);
        setDroneName('');
        setStatus('IN_TRANSIT');
        setBatteryLife(100);
        setWarehouseId('');
        setDroneModelId('');
        setSuccessMessage('');
    };

    const handleRegisterDrone = (e) => {
        e.preventDefault();

        const newDrone = {
            name: droneName,
            status,
            batteryLife,
            warehouse: warehouses.find(warehouse => warehouse.id === parseInt(warehouseId)),
            model: droneModels.find(model => model.id === parseInt(droneModelId)),
        };

        DroneService.registerDrone(newDrone)
            .then(() => {
                setSuccessMessage('Drone successfully registered!');
                setDrones([...drones, { ...newDrone, id: drones.length + 1 }]);
                setTimeout(handleCloseModal, 1500);  // Close the modal after success
            })
            .catch((err) => {
                setError('Error registering drone: ' + err.message);
            });
    };

    return (
        <Container className='mt-4'>
            <h2 className="text-center">Drones List</h2>

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
                    <FaPlus /> Register New Drone
                </Button>
            </div>

            <Row className="mt-4">
                {drones.map(drone => (
                    <Col key={drone.id} xs={12} sm={6} md={4} lg={3} className="mb-4">
                        <Card>
                            <Card.Body>
                                <Card.Title>{drone.name}</Card.Title>
                                <Card.Text>
                                    <strong>ID:</strong> {drone.id} <br />
                                    <strong>Status:</strong> {drone.status} <br />
                                    <strong>Battery Life:</strong> {drone.batteryLife}% <br />
                                    <strong>Location:</strong> {drone.lastLatitudeLocation}, {drone.lastLongitudeLocation} <br />
                                    <strong>Last Maintenance:</strong> {new Date(drone.lastMaintenanceDate).toLocaleString()} <br />
                                    <strong>Warehouse:</strong> {drone.warehouse?.name || 'N/A'} <br />
                                    <strong>Drone Model:</strong> {drone.model?.manufacturer || 'N/A'} <br />
                                    <strong>Connected to System:</strong> {drone.connectedToSystem ? 'Yes' : 'No'} <br />
                                    <strong>Alerts:</strong> {drone.alerts?.length > 0 ? drone.alerts.join(', ') : 'No alerts'}
                                </Card.Text>
                            </Card.Body>
                        </Card>
                    </Col>
                ))}
            </Row>

            {/* Modal for Registering Drone */}
            <Modal show={showModal} onHide={handleCloseModal} centered>
                <Modal.Header closeButton>
                    <Modal.Title>Register a New Drone</Modal.Title>
                </Modal.Header>
                <Modal.Body>
                    <Form onSubmit={handleRegisterDrone}>
                        <Form.Group controlId="droneName">
                            <Form.Label>Drone Name</Form.Label>
                            <Form.Control
                                type="text"
                                placeholder="Enter drone name"
                                value={droneName}
                                onChange={(e) => setDroneName(e.target.value)}
                                required
                            />
                        </Form.Group>

                        <Form.Group controlId="droneStatus" className="mt-3">
                            <Form.Label>Status</Form.Label>
                            <Form.Control as="select" value={status} onChange={(e) => setStatus(e.target.value)} required>
                                <option value="IN_TRANSIT">In Transit</option>
                                <option value="IDLE">Idle</option>
                                <option value="GOING_TO_WAREHOUSE">Going to Warehouse</option>
                            </Form.Control>
                        </Form.Group>

                        <Form.Group controlId="batteryLife" className="mt-3">
                            <Form.Label>Battery Life (%)</Form.Label>
                            <Form.Control
                                type="number"
                                value={batteryLife}
                                onChange={(e) => setBatteryLife(e.target.value)}
                                required
                            />
                        </Form.Group>

                        <Form.Group controlId="warehouse" className="mt-3">
                            <Form.Label>Select Warehouse</Form.Label>
                            <Form.Control as="select" value={warehouseId} onChange={(e) => setWarehouseId(e.target.value)} required>
                                <option value="">Select Warehouse</option>
                                {warehouses.map(warehouse => (
                                    <option key={warehouse.id} value={warehouse.id}>{warehouse.name}</option>
                                ))}
                            </Form.Control>
                        </Form.Group>

                        <Form.Group controlId="droneModel" className="mt-3">
                            <Form.Label>Select Drone Model</Form.Label>
                            <Form.Control as="select" value={droneModelId} onChange={(e) => setDroneModelId(e.target.value)} required>
                                <option value="">Select Drone Model</option>
                                {droneModels.map(model => (
                                    <option key={model.id} value={model.id}>{model.manufacturer}</option>
                                ))}
                            </Form.Control>
                        </Form.Group>

                        <Button variant="success" type="submit" className="mt-3 w-100">
                            Register Drone
                        </Button>
                    </Form>
                </Modal.Body>
            </Modal>
        </Container>
    );
}

export default DroneList;