import React, { useState, useEffect } from 'react';
import DroneService from '../services/DroneService';
import { Container, Card, Row, Col, Button, Spinner, Alert, Modal, Form } from 'react-bootstrap';
import { FaPlus } from 'react-icons/fa';

function DroneList() {
    const [drones, setDrones] = useState([]);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState(null);
    const [showModal, setShowModal] = useState(false);
    const [droneName, setDroneName] = useState('');
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
    }, []);

    const handleRegisterClick = () => {
        setShowModal(true);
    };

    const handleCloseModal = () => {
        setShowModal(false);
        setDroneName('');
        setSuccessMessage('');
    };

    const handleRegisterDrone = (e) => {
        e.preventDefault();
        DroneService.registerDrone({ name: droneName })
            .then(() => {
                setSuccessMessage('Drone successfully registered!');
                setDrones([...drones, { name: droneName, id: drones.length + 1 }]);
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
                                    <strong>ID:</strong> {drone.id}
                                </Card.Text>
                            </Card.Body>
                        </Card>
                    </Col>
                ))}
            </Row>

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
                        <Button variant="success" type="submit" className="mt-3" block>
                            Register Drone
                        </Button>
                    </Form>
                </Modal.Body>
            </Modal>
        </Container>
    );
}

export default DroneList;