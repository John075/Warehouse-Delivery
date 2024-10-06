import React from 'react';
import { Container, Row, Col, Card } from 'react-bootstrap';
import { FaCogs, FaTruck, FaBoxOpen, FaWarehouse, FaRobot } from 'react-icons/fa';

function WelcomePage() {
    const cardStyle = {
        borderRadius: '10px',
        padding: '20px',
        margin: '15px 0',
        transition: 'transform 0.3s ease-in-out',
        boxShadow: '0 4px 15px rgba(0, 0, 0, 0.1)',
        background: '#fff',
    };

    const titleStyle = {
        fontSize: '1.75rem',
        fontWeight: '600',
        color: '#343a40',
        marginTop: '5px',
    };

    const subtitleStyle = {
        fontSize: '1.25rem',
        color: '#6c757d',
    };

    const iconStyle = {
        fontSize: '2rem',
        marginBottom: '10px',
        color: '#007bff',
    };

    const sectionTitle = {
        fontSize: '1.5rem',
        fontWeight: '600',
        marginTop: '5px',
    };

    const additionalTextStyle = {
        fontSize: '1rem',
        marginTop: '10px',
        color: '#6c757d',
    };

    return (
        <div style={{ backgroundColor: '#f4f7fa', minHeight: '100vh', padding: '40px 0' }}>
            <Container className="text-center">
                <Row>
                    <Col>
                        <h1 style={titleStyle}>Welcome to Your Delivery Control Panel</h1>
                        <p style={subtitleStyle}>
                            Manage your drones, packages, warehouses, and models easily. No technical knowledge needed!
                        </p>
                    </Col>
                </Row>

                <Row className="mt-4">
                    <Col xs={12} md={6}>
                        <Card style={cardStyle} className="hoverable-card">
                            <FaTruck style={iconStyle} />
                            <h2 style={sectionTitle}>Manage Drones</h2>
                            <p style={additionalTextStyle}>
                                View and manage your drones. Check their status, assign them deliveries, and monitor their location in real-time.
                            </p>
                        </Card>
                    </Col>
                    <Col xs={12} md={6}>
                        <Card style={cardStyle} className="hoverable-card">
                            <FaBoxOpen style={iconStyle} />
                            <h2 style={sectionTitle}>Manage Packages</h2>
                            <p style={additionalTextStyle}>
                                Keep track of all your deliveries. Assign packages to drones, check their delivery status, and track their destination.
                            </p>
                        </Card>
                    </Col>
                </Row>

                <Row className="mt-4">
                    <Col xs={12} md={6}>
                        <Card style={cardStyle} className="hoverable-card">
                            <FaRobot style={iconStyle} />
                            <h2 style={sectionTitle}>Drone Models</h2>
                            <p style={additionalTextStyle}>
                                Keep your fleet up-to-date. Add new drone models or update existing ones with their latest capabilities.
                            </p>
                        </Card>
                    </Col>
                    <Col xs={12} md={6}>
                        <Card style={cardStyle} className="hoverable-card">
                            <FaWarehouse style={iconStyle} />
                            <h2 style={sectionTitle}>Manage Warehouses</h2>
                            <p style={additionalTextStyle}>
                                Organize and update your warehouse locations. Track their capacity and available space for your deliveries.
                            </p>
                        </Card>
                    </Col>
                </Row>

                <Row className="mt-5">
                    <Col className="text-center">
                        <FaCogs size={60} className="text-muted" />
                        <p style={{ ...additionalTextStyle, marginTop: '20px' }}>
                            Easily manage your fleet and deliveries with our simple tools. No complex setups, just straightforward management at your fingertips.
                        </p>
                    </Col>
                </Row>
            </Container>
        </div>
    );
}

export default WelcomePage;