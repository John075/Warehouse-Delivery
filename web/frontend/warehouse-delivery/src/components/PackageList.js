import React, { useState, useEffect } from 'react';
import PackageService from '../services/PackageService';
import { Container, Table, Button, Spinner, Alert, Modal, Form } from 'react-bootstrap';
import { FaPlus } from 'react-icons/fa';

function PackageList() {
    const [packages, setPackages] = useState([]);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState(null);
    const [showModal, setShowModal] = useState(false);
    const [packageName, setPackageName] = useState('');
    const [assignedDrone, setAssignedDrone] = useState('');
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
    }, []);

    const handleRegisterClick = () => {
        setShowModal(true);
    };

    const handleCloseModal = () => {
        setShowModal(false);
        setPackageName('');
        setAssignedDrone('');
        setStatus('');
        setDestination({ street: '', city: '', state: '', zipCode: '', country: '' });
        setPriority(0);
        setSuccessMessage('');
    };

    const handleRegisterPackage = (e) => {
        e.preventDefault();
        const newPackage = {
            name: packageName,
            assignedDrone: assignedDrone || null,
            status,
            destination,
            priority,
        };

        PackageService.registerPackage(newPackage)
            .then(() => {
                setSuccessMessage('Package successfully registered!');
                setPackages([...packages, { ...newPackage, id: packages.length + 1 }]);
                setTimeout(handleCloseModal, 1500);  // Close the modal after success
            })
            .catch((err) => {
                setError('Error registering package: ' + err.message);
            });
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
                    <th>Status</th>
                    <th>Destination</th>
                    <th>Priority</th>
                </tr>
                </thead>
                <tbody>
                {packages.map(pkg => (
                    <tr key={pkg.id}>
                        <td>{pkg.id}</td>
                        <td>{pkg.name}</td>
                        <td>{pkg.assignedDrone ? pkg.assignedDrone.name : 'Unassigned'}</td>
                        <td>{pkg.status}</td>
                        <td>{`${pkg.destination.street}, ${pkg.destination.city}, ${pkg.destination.state}`}</td>
                        <td>{pkg.priority}</td>
                    </tr>
                ))}
                </tbody>
            </Table>

            <Modal show={showModal} onHide={handleCloseModal} centered>
                <Modal.Header closeButton>
                    <Modal.Title>Register a New Package</Modal.Title>
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
                                type="text"
                                placeholder="Enter drone ID (optional)"
                                value={assignedDrone}
                                onChange={(e) => setAssignedDrone(e.target.value)}
                            />
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
                                type="text"
                                placeholder="Street"
                                value={destination.street}
                                onChange={(e) => setDestination({ ...destination, street: e.target.value })}
                            />
                            <Form.Control
                                type="text"
                                placeholder="City"
                                value={destination.city}
                                onChange={(e) => setDestination({ ...destination, city: e.target.value })}
                                className="mt-2"
                            />
                            <Form.Control
                                type="text"
                                placeholder="State"
                                value={destination.state}
                                onChange={(e) => setDestination({ ...destination, state: e.target.value })}
                                className="mt-2"
                            />
                            <Form.Control
                                type="text"
                                placeholder="Zip Code"
                                value={destination.zipCode}
                                onChange={(e) => setDestination({ ...destination, zipCode: e.target.value })}
                                className="mt-2"
                            />
                            <Form.Control
                                type="text"
                                placeholder="Country"
                                value={destination.country}
                                onChange={(e) => setDestination({ ...destination, country: e.target.value })}
                                className="mt-2"
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
                            Register Package
                        </Button>
                    </Form>
                </Modal.Body>
            </Modal>
        </Container>
    );
}

export default PackageList;