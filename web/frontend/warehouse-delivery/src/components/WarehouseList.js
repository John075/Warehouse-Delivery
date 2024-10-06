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
        street: '',
        city: '',
        state: '',
        zipCode: '',
        country: '',
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
            street: '',
            city: '',
            state: '',
            zipCode: '',
            country: '',
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
            latitude: parseFloat(newWarehouse.latitude),
            longitude: parseFloat(newWarehouse.longitude),
            address: {
                street: newWarehouse.street,
                city: newWarehouse.city,
                state: newWarehouse.state,
                zipCode: newWarehouse.zipCode,
                country: newWarehouse.country,
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
                    <th>Address</th>
                    <th>Capacity</th>
                </tr>
                </thead>
                <tbody>
                {warehouses.map(warehouse => (
                    <tr key={warehouse.id}>
                        <td>{warehouse.id}</td>
                        <td>{warehouse.name}</td>
                        <td>{warehouse.latitude}</td>
                        <td>{warehouse.longitude}</td>
                        <td>
                            {warehouse.address.street}, {warehouse.address.city}, {warehouse.address.state}, {warehouse.address.zipCode}, {warehouse.address.country}
                        </td>
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
                        <Form.Group controlId="street" className="mt-3">
                            <Form.Label>Street</Form.Label>
                            <Form.Control
                                type="text"
                                placeholder="Enter street"
                                name="street"
                                value={newWarehouse.street}
                                onChange={handleInputChange}
                                required
                            />
                        </Form.Group>
                        <Form.Group controlId="city" className="mt-3">
                            <Form.Label>City</Form.Label>
                            <Form.Control
                                type="text"
                                placeholder="Enter city"
                                name="city"
                                value={newWarehouse.city}
                                onChange={handleInputChange}
                                required
                            />
                        </Form.Group>
                        <Form.Group controlId="state" className="mt-3">
                            <Form.Label>State</Form.Label>
                            <Form.Control
                                type="text"
                                placeholder="Enter state"
                                name="state"
                                value={newWarehouse.state}
                                onChange={handleInputChange}
                                required
                            />
                        </Form.Group>
                        <Form.Group controlId="zipCode" className="mt-3">
                            <Form.Label>Zip Code</Form.Label>
                            <Form.Control
                                type="text"
                                placeholder="Enter zip code"
                                name="zipCode"
                                value={newWarehouse.zipCode}
                                onChange={handleInputChange}
                                required
                            />
                        </Form.Group>
                        <Form.Group controlId="country" className="mt-3">
                            <Form.Label>Country</Form.Label>
                            <Form.Control
                                type="text"
                                placeholder="Enter country"
                                name="country"
                                value={newWarehouse.country}
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