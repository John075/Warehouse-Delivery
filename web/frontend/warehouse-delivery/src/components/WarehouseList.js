import React, { useState, useEffect } from 'react';
import WarehouseService from '../services/WarehouseService';
import { Container, Table, Button, Spinner, Alert, Modal, Form } from 'react-bootstrap';
import { FaPlus } from 'react-icons/fa';

function WarehouseList() {
    const [warehouses, setWarehouses] = useState([]);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState(null);
    const [showModal, setShowModal] = useState(false);
    const [newWarehouse, setNewWarehouse] = useState({
        name: '',
        latitude: '',
        longitude: '',
        capacity: ''
    });
    const [successMessage, setSuccessMessage] = useState('');

    useEffect(() => {
        WarehouseService.getWarehouses().then((response) => {
            setWarehouses(response.data);
            setLoading(false);
        }).catch(error => {
            console.error("Error fetching warehouses:", error);
            setError("There was an error fetching the warehouse data.");
            setLoading(false);
        });
    }, []);

    const handleRegisterClick = () => {
        setShowModal(true);
    };

    const handleCloseModal = () => {
        setShowModal(false);
        setNewWarehouse({
            name: '',
            latitude: '',
            longitude: '',
            capacity: ''
        });
        setSuccessMessage('');
    };

    const handleInputChange = (e) => {
        const { name, value } = e.target;
        setNewWarehouse({ ...newWarehouse, [name]: value });
    };

    const handleRegisterWarehouse = (e) => {
        e.preventDefault();
        const warehouseData = {
            name: newWarehouse.name,
            address: {
                latitude: parseFloat(newWarehouse.latitude),
                longitude: parseFloat(newWarehouse.longitude),
            },
            capacity: parseInt(newWarehouse.capacity, 10),
        };
        WarehouseService.registerWarehouse(warehouseData)
            .then(() => {
                setSuccessMessage('Warehouse successfully registered!');
                setWarehouses([...warehouses, { ...warehouseData, id: warehouses.length + 1 }]);
                setTimeout(handleCloseModal, 1500);
            })
            .catch((err) => {
                setError('Error registering warehouse: ' + err.message);
            });
    };

    return (
        <Container className='mt-4'>
            <h2 className="text-center">Warehouse List</h2>

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
                    <FaPlus /> Register New Warehouse
                </Button>
            </div>

            <Table striped bordered hover className="mt-4">
                <thead>
                <tr>
                    <th>ID</th>
                    <th>Name</th>
                    <th>Latitude</th>
                    <th>Longitude</th>
                    <th>Capacity</th>
                </tr>
                </thead>
                <tbody>
                {warehouses.map(warehouse => (
                    <tr key={warehouse.id}>
                        <td>{warehouse.id}</td>
                        <td>{warehouse.name}</td>
                        <td>{warehouse.address ? warehouse.address.latitude : "Latitude N/A"}</td>
                        <td>{warehouse.address ? warehouse.address.longitude : "Longitude N/A"}</td>
                        <td>{warehouse.capacity}</td>
                    </tr>
                ))}
                </tbody>
            </Table>

            <Modal show={showModal} onHide={handleCloseModal} centered>
                <Modal.Header closeButton>
                    <Modal.Title>Register a New Warehouse</Modal.Title>
                </Modal.Header>
                <Modal.Body>
                    <Form onSubmit={handleRegisterWarehouse}>
                        <Form.Group controlId="name">
                            <Form.Label>Warehouse Name</Form.Label>
                            <Form.Control
                                type="text"
                                placeholder="Enter warehouse name"
                                name="name"
                                value={newWarehouse.name}
                                onChange={handleInputChange}
                                required
                            />
                        </Form.Group>
                        <Form.Group controlId="latitude" className="mt-3">
                            <Form.Label>Latitude</Form.Label>
                            <Form.Control
                                type="number"
                                step="any"
                                placeholder="Enter latitude"
                                name="latitude"
                                value={newWarehouse.latitude}
                                onChange={handleInputChange}
                                required
                            />
                        </Form.Group>
                        <Form.Group controlId="longitude" className="mt-3">
                            <Form.Label>Longitude</Form.Label>
                            <Form.Control
                                type="number"
                                step="any"
                                placeholder="Enter longitude"
                                name="longitude"
                                value={newWarehouse.longitude}
                                onChange={handleInputChange}
                                required
                            />
                        </Form.Group>
                        <Form.Group controlId="capacity" className="mt-3">
                            <Form.Label>Capacity</Form.Label>
                            <Form.Control
                                type="number"
                                placeholder="Enter capacity"
                                name="capacity"
                                value={newWarehouse.capacity}
                                onChange={handleInputChange}
                                required
                            />
                        </Form.Group>
                        <Button variant="success" type="submit" className="mt-3 w-100">
                            Register Warehouse
                        </Button>
                    </Form>
                </Modal.Body>
            </Modal>
        </Container>
    );
}

export default WarehouseList;