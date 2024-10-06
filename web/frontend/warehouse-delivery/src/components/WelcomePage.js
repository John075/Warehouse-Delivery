// src/components/WelcomePage.js
import React from 'react';
import { Container, Row, Col, Button } from 'react-bootstrap';
import { FaCogs, FaTruck, FaBoxOpen } from 'react-icons/fa';

function WelcomePage() {
    return (
        <div className="welcome-page">
            <Container className="text-center mt-5">
                <Row>
                    <Col>
                        <h1 className="welcome-title">Welcome to the Warehouse Delivery Admin Panel</h1>
                        <p className="lead welcome-subtitle">Manage your drones and packages efficiently with our system.</p>
                    </Col>
                </Row>

                <Row className="mt-5">
                    <Col className="d-flex justify-content-center">
                        <Button href="/drones" variant="primary" size="lg" className="welcome-btn mr-3 shadow-lg">
                            <FaTruck className="mr-2" /> View Drones
                        </Button>
                    </Col>
                    <Col className="d-flex justify-content-center">
                        <Button href="/packages" variant="secondary" size="lg" className="welcome-btn shadow-lg">
                            <FaBoxOpen className="mr-2" /> View Packages
                        </Button>
                    </Col>
                </Row>

                <Row className="mt-5">
                    <Col className="text-center">
                        <FaCogs size={60} className="text-muted" />
                        <p className="text-muted mt-3">Streamlining your delivery management process.</p>
                    </Col>
                </Row>
            </Container>
        </div>
    );
}

export default WelcomePage;