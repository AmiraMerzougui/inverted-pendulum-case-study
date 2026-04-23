# README for Inverted Pendulum Case Study

## 1. Project Overview
This project explores the dynamics and control of an inverted pendulum system, a classical problem in control engineering. The focus is on developing a robust control solution that ensures stability and performance of the pendulum in an upright position through various techniques such as PID and LQR control.

## 2. System Architecture
The system is represented using state-space equations, which provide a comprehensive understanding of the system dynamics. The state vector can be defined as:
\[ x = \begin{bmatrix} \theta \\ \dot{\theta} \end{bmatrix} \]
Where \( \theta \) is the angle of the pendulum from the vertical axis and \( \dot{\theta} \) is the angular velocity.

The state equations are given by:
\[ \dot{x} = Ax + Bu \]
Where:
- \( A \) is the system matrix 
- \( B \) is the control input matrix

## 3. Requirements Definition
Performance metrics include:
- **Rise Time**: Less than 2 seconds
- **Settling Time**: Less than 5 seconds
- **Overshoot**: Less than 10%
- **Steady-State Error**: Less than 5%

## 4. Modeling Approach
The initial nonlinear model of the inverted pendulum is linearized around the equilibrium point using Taylor series expansion. The linearized model allows for easier analysis and the design of control strategies:
\[ \dot{x} = A_{lin} x + B_{lin} u \]

## 5. Control Strategy
A comparison between PID and LQR control strategies:
### PID Control
- **Advantages**: Simplicity, ease of implementation.
- **Disadvantages**: May not achieve optimal control, susceptible to external disturbances.

### LQR Control
- **Advantages**: Optimal control solution minimizing the cost function.
- **Disadvantages**: Requires accurate system modeling.

### Justification
LQR is preferred for its ability to minimize the energy consumption and achieve better control performance.

## 6. Simulation & Validation
A multi-fidelity approach was adopted, combining high-fidelity simulations with low-cost models to validate system performance across a range of scenarios.

## 7. Results & Insights
Results indicate that the LQR controller significantly reduces overshoot and settling time compared to the PID controller under identical conditions. Detailed tables of performance metrics are included in the attachments.

## 8. System Limitations
Potential limitations include:
- Sensitivity to parameter variations.
- Model inaccuracies can lead to degraded performance.

## 9. Path to Real Implementation
Future steps include:
- Prototyping on a physical inverted pendulum set-up.
- Iterative testing and refinement based on real-world results.

## 10. How to Run
To execute the model:
1. Clone the repository:  `git clone https://github.com/AmiraMerzougui/inverted-pendulum-case-study.git`
2. Navigate to the project directory.
3. Run simulations using `python simulate.py`

For further instructions, refer to the documentation located in the `docs` folder.