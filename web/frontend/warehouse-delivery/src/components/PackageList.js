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
        setSuccessMessage('');
    };

    const handleRegisterPackage = (e) => {
        e.preventDefault();
        PackageService.registerPackage({ name: packageName })
            .then(() => {
                setSuccessMessage('Package successfully registered!');
                setPackages([...packages, { name: packageName, id: packages.length + 1 }]);
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
                </tr>
                </thead>
                <tbody>
                {packages.map(pkg => (
                    <tr key={pkg.id}>
                        <td>{pkg.id}</td>
                        <td>{pkg.name}</td>
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
                        <Button variant="success" type="submit" className="mt-3" block>
                            Register Package
                        </Button>
                    </Form>
                </Modal.Body>
            </Modal>
        </Container>
    );
}

export default PackageList;