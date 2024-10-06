import { NavLink } from 'react-router-dom';

const HeaderComponent = () => {
    return (
        <div>
            <header>
                <nav className="navbar navbar-expand-lg navbar-dark bg-dark shadow-sm sticky-top">
                    <div className="container">
                        <a className="navbar-brand font-weight-bold" href="/">
                            Warehouse Delivery
                        </a>
                        <button
                            className="navbar-toggler"
                            type="button"
                            data-toggle="collapse"
                            data-target="#navbarNav"
                            aria-controls="navbarNav"
                            aria-expanded="false"
                            aria-label="Toggle navigation"
                        >
                            <span className="navbar-toggler-icon"></span>
                        </button>
                        <div className="collapse navbar-collapse" id="navbarNav">
                            <ul className="navbar-nav ml-auto">
                                <li className="nav-item">
                                    <NavLink
                                        className={({ isActive }) => "nav-link" + (isActive ? " active" : "")}
                                        to="/drones"
                                    >
                                        Drones
                                    </NavLink>
                                </li>
                                <li className="nav-item">
                                    <NavLink
                                        className={({ isActive }) => "nav-link" + (isActive ? " active" : "")}
                                        to="/packages"
                                    >
                                        Packages
                                    </NavLink>
                                </li>
                                <li className="nav-item">
                                    <NavLink
                                        className={({ isActive }) => "nav-link" + (isActive ? " active" : "")}
                                        to="/drone-models"
                                    >
                                        Drone Models
                                    </NavLink>
                                </li>
                                <li className="nav-item">
                                    <NavLink
                                        className={({ isActive }) => "nav-link" + (isActive ? " active" : "")}
                                        to="/warehouses"
                                    >
                                        Warehouses
                                    </NavLink>
                                </li>
                            </ul>
                        </div>
                    </div>
                </nav>
            </header>
        </div>
    );
};

export default HeaderComponent;