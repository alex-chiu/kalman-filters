#!/usr/bin/python3.11
from typing import Callable, Tuple
from pathlib import Path

import argparse
import numpy as np


def runge_kutta_45(t: float, y: np.ndarray, h: float, deriv_func: Callable) -> Tuple[np.ndarray, float]:
    """
    Runge-Kutta 4(5) method for numerical integration.

    Parameters:
        t (float): Current time.
        y (np.ndarray): Current state vector.
        h (float): Step size.
        deriv_func (Callable): Function representing the system of ODEs.

    Returns:
        Tuple[np.ndarray, float]: Tuple containing the updated state vector and error estimate.
    """
    k1 = h * deriv_func(t, y)
    k2 = deriv_func(t + 0.25 * h, y + 0.25 * k1)
    np.multiply(k2, h, out=k2)
    k3 = deriv_func(t + 3/8 * h, y + 3/32 * k1 + 9/32 * k2)
    np.multiply(k3, h, out=k3)
    k4 = deriv_func(t + 12/13 * h, y + 1932/2197 *
                    k1 - 7200/2197 * k2 + 7296/2197 * k3)
    np.multiply(k4, h, out=k4)
    k5 = deriv_func(t + h, y + 439/216 * k1 - 8 *
                    k2 + 3680/513 * k3 - 845/4104 * k4)
    np.multiply(k5, h, out=k5)
    k6 = deriv_func(t + 0.5 * h, y - 8/27 * k1 + 2 * k2 -
                    3544/2565 * k3 + 1859/4104 * k4 - 11/40 * k5)
    np.multiply(k6, h, out=k6)

    y_new = y + 25/216 * k1 + 1408/2565 * k3 + 2197/4104 * k4 - 1/5 * k5

    error = np.linalg.norm(
        (1/360 * k1 - 128/4275 * k3 - 2197/75240 * k4 + 1/50 * k5 + 2/55 * k6))

    return y_new, error


def two_body_deriv(t: float, state: np.ndarray) -> np.ndarray:
    """
    Calculate the derivative of the state vector for a two-body problem.

    Args:
        t (float): Current time.
        state (np.ndarray): Current state vector containing position and velocity components.

    Returns:
        np.ndarray: Derivative of the state vector containing velocity and acceleration components.
    """
    mu = 398600.4418  # Earth's gravitational parameter in km^3/s^2

    x, y, z, vx, vy, vz = state

    r = np.sqrt(x**2 + y**2 + z**2)
    ax = -mu * x / r**3
    ay = -mu * y / r**3
    az = -mu * z / r**3

    return np.array([vx, vy, vz, ax, ay, az])


def propagate_orbit(initial_state: np.ndarray, start_time: float, end_time: float, time_step: float) -> tuple[np.ndarray, np.ndarray]:
    """
    Propagate the orbit of a two-body system over a specified time range.

    Args:
        initial_state (np.ndarray): Initial state vector containing position and velocity components.
        start_time (float): Start time of the propagation.
        end_time (float): End time of the propagation.
        time_step (float): Time step for the propagation.

    Returns:
        tuple[np.ndarray, np.ndarray]: Tuple containing the time array and the array of state vectors at each time step.
    """
    times = np.arange(start_time, end_time, time_step)
    states = np.empty((len(times), *initial_state.shape))
    states[0] = initial_state

    for i, t in enumerate(times[:-1]):
        y, error = runge_kutta_45(t, states[i], time_step, two_body_deriv)
        states[i + 1] = y

    return times, states


def write_to_csv(file_path: Path, times: np.ndarray, states: np.ndarray) -> None:
    """
    Write the times and states data to a CSV file.

    Args:
        file_path (Path): Path to the output CSV file.
        times (np.ndarray): Array of times.
        states (np.ndarray): Array of states.

    Returns:
        None
    """
    data = np.column_stack((times, states))
    header = "Time,X,Y,Z,VX,VY,VZ"
    np.savetxt(file_path, data, delimiter=",", header=header, comments='')


def main() -> None:
    """
    Entry point of the program for propagating a two-body orbit and saving state vectors to CSV.

    Returns:
        None
    """
    # Define initial state
    initial_state = np.array([6711.0, 30.0, -60.0, 1.0, 7.7, -1.0])

    # Propagate orbit for 1 orbit (24 hours)
    start_time = 0
    end_time = 24 * 3600  # 24 hours in seconds
    time_step = 60  # Time step in seconds

    times, states = propagate_orbit(
        initial_state, start_time, end_time, time_step)

    # Make parser for csv destination
    parser = argparse.ArgumentParser(
        description="Propagate a two-body orbit and save state vectors to CSV.")
    parser.add_argument('--output', '-o', type=str,
                        default="orbit_states.csv", help="Path to the output CSV file")
    args = parser.parse_args()

    # Write states to CSV
    write_to_csv(Path(args.output), times, states)


if __name__ == "__main__":
    main()
