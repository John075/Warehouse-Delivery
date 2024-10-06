import React, { useState, useEffect } from 'react';
import DroneModelService from '../services/DroneModelService';
import { Container, Row, Col, Card, Spinner, Alert, Modal, Form, Button } from 'react-bootstrap';
import { FaPlus } from 'react-icons/fa';

function DroneModelList() {
    const [droneModels, setDroneModels] = useState([]);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState(null);
    const [showModal, setShowModal] = useState(false);
    const [newDroneModel, setNewDroneModel] = useState({
        manufacturer: '',
        maxSpeed: '',
        recommendedSpeed: '',
        batteryCapacity: '',
        maxTripLength: '',
        maxPayloadCapacity: '',
    });
    const [successMessage, setSuccessMessage] = useState('');

    useEffect(() => {
        DroneModelService.getDroneModels().then((response) => {
            setDroneModels(response.data);
            setLoading(false);
        }).catch(error => {
            console.error("Error fetching drone models:", error);
            setError("There was an error fetching the drone models.");
            setLoading(false);
        });
    }, []);

    const handleRegisterClick = () => {
        setShowModal(true);
    };

    const handleCloseModal = () => {
        setShowModal(false);
        setNewDroneModel({
            manufacturer: '',
            maxSpeed: '',
            recommendedSpeed: '',
            batteryCapacity: '',
            maxTripLength: '',
            maxPayloadCapacity: '',
        });
        setSuccessMessage('');
    };

    const handleInputChange = (e) => {
        const { name, value } = e.target;
        setNewDroneModel({ ...newDroneModel, [name]: value });
    };

    const handleRegisterDroneModel = (e) => {
        e.preventDefault();
        DroneModelService.registerDroneModel(newDroneModel)
            .then(() => {
                setSuccessMessage('Drone model successfully registered!');
                setDroneModels([...droneModels, { ...newDroneModel, id: droneModels.length + 1 }]);
                setTimeout(handleCloseModal, 1500);  // Close the modal after success
            })
            .catch((err) => {
                setError('Error registering drone model: ' + err.message);
            });
    };

    return (
        <Container className='mt-4'>
            <h2 className="text-center">Drone Models</h2>

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
                    <FaPlus /> Register New Drone Model
                </Button>
            </div>

            <Row className="mt-4">
                {droneModels.map(droneModel => (
                    <Col key={droneModel.id} xs={12} sm={6} md={4} lg={3} className="mb-4">
                        <Card>
                            <Card.Body>
                                <Card.Title>{droneModel.manufacturer}</Card.Title>
                                <Card.Text>
                                    <strong>Max Speed:</strong> {droneModel.maxSpeed} km/h<br />
                                    <strong>Recommended Speed:</strong> {droneModel.recommendedSpeed} km/h<br />
                                    <strong>Battery Capacity:</strong> {droneModel.batteryCapacity} mAh<br />
                                    <strong>Max Trip Length:</strong> {droneModel.maxTripLength} km<br />
                                    <strong>Max Payload Capacity:</strong> {droneModel.maxPayloadCapacity} kg
                                </Card.Text>
                            </Card.Body>
                        </Card>
                    </Col>
                ))}
            </Row>

            <Modal show={showModal} onHide={handleCloseModal} centered>
                <Modal.Header closeButton>
                    <Modal.Title>Register a New Drone Model</Modal.Title>
                </Modal.Header>
                <Modal.Body>
                    <Form onSubmit={handleRegisterDroneModel}>
                        <Form.Group controlId="manufacturer">
                            <Form.Label>Manufacturer</Form.Label>
                            <Form.Control
                                type="text"
                                placeholder="Enter manufacturer"
                                name="manufacturer"
                                value={newDroneModel.manufacturer}
                                onChange={handleInputChange}
                                required
                            />
                        </Form.Group>
                        <Form.Group controlId="maxSpeed" className="mt-3">
                            <Form.Label>Max Speed (km/h)</Form.Label>
                            <Form.Control
                                type="number"
                                placeholder="Enter max speed"
                                name="maxSpeed"
                                value={newDroneModel.maxSpeed}
                                onChange={handleInputChange}
                                required
                            />
                        </Form.Group>
                        <Form.Group controlId="recommendedSpeed" className="mt-3">
                            <Form.Label>Recommended Speed (km/h)</Form.Label>
                            <Form.Control
                                type="number"
                                placeholder="Enter recommended speed"
                                name="recommendedSpeed"
                                value={newDroneModel.recommendedSpeed}
                                onChange={handleInputChange}
                                required
                            />
                        </Form.Group>
                        <Form.Group controlId="batteryCapacity" className="mt-3">
                            <Form.Label>Battery Capacity (mAh)</Form.Label>
                            <Form.Control
                                type="number"
                                placeholder="Enter battery capacity"
                                name="batteryCapacity"
                                value={newDroneModel.batteryCapacity}
                                onChange={handleInputChange}
                                required
                            />
                        </Form.Group>
                        <Form.Group controlId="maxTripLength" className="mt-3">
                            <Form.Label>Max Trip Length (km)</Form.Label>
                            <Form.Control
                                type="number"
                                placeholder="Enter max trip length"
                                name="maxTripLength"
                                value={newDroneModel.maxTripLength}
                                onChange={handleInputChange}
                                required
                            />
                        </Form.Group>
                        <Form.Group controlId="maxPayloadCapacity" className="mt-3">
                            <Form.Label>Max Payload Capacity (kg)</Form.Label>
                            <Form.Control
                                type="number"
                                placeholder="Enter max payload capacity"
                                name="maxPayloadCapacity"
                                value={newDroneModel.maxPayloadCapacity}
                                onChange={handleInputChange}
                                required
                            />
                        </Form.Group>
                        <Button variant="success" type="submit" className="mt-3 w-100">
                            Register Drone Model
                        </Button>
                    </Form>
                </Modal.Body>
            </Modal>
        </Container>
    );
}

export default DroneModelList;