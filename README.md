# Data-Driven Control Systems: Design and Application of Data-Driven Controllers

## Introduction
This project focuses on the design and implementation of self-tuning PID control using recorded data for systems with strong nonlinearity. While traditional PID controllers with fixed parameters can manage systems with mild nonlinearity, those with significant nonlinear characteristics require dynamic parameter adjustment for effective control. The project explores an online data-driven control scheme and an offline data-driven fictitious reference iterative tuning (DD FRIT) control scheme, aiming to reduce computational and data requirements compared to complex control algorithms like neural networks or genetic algorithms.

## Online Data-Driven Control Scheme
This approach dynamically adapts PID parameters in response to control errors. The updated parameters are stored in a database, which is concurrently optimized by removing redundant data points. This strategy significantly reduces the computational load and memory storage requirements.

## Offline DD FRIT Control Scheme
In contrast to the online method, the DD FRIT control scheme operates offline. It generates a reference signal from initial closed-loop data and control parameters, and then calculates optimal PID parameters using this reference. This method does not add new data to the database in real time, differing significantly from the online approach.

## Experiment on Tank System
The DD FRIT method was applied to a Tank System for level control. The system's non-metallic nature meant only cold water flow was controllable. The experiment involved tuning a local controller to linearize the cold flow input and output, followed by tuning the main controller to achieve acceptable control performance.

### Key Observations:
- Initial tuning of the local PID controller was crucial for linearizing the system's response.
- The main controller was adjusted to achieve satisfactory results.
- The data-driven FRIT control scheme enhanced the system's response by providing a stronger input signal, thus reducing the rise time.
- Differential gain (KD) was set to zero due to high-frequency noise in the water level sensor, as KD could amplify this noise.

## References

[1] T. Yamamoto, K. Takao and T. Yamada, "Design of a Data-Driven PID Controller," in _IEEE Transactions on Control Systems Technology_, vol. 17, no. 1, pp. 29-39, Jan. 2009. doi: 10.1109/TCST.2008.921808

[2] Wakitani, Shin & Yamamoto, Toru & Gopaluni, Bhushan. (2019). Design and Application of a Database-Driven PID Controller with Data-Driven Updating Algorithm. Industrial & Engineering Chemistry Research. 10.1021/acs.iecr.9b00704.

[3] S. Wakitani and T. Yamamoto, "A learning algorithm for a data-driven controller based on fictitious reference iterative tuning," _2016 International Joint Conference on Neural Networks (IJCNN)_, Vancouver, BC, 2016, pp. 4908-4913. doi: 10.1109/IJCNN.2016.7727845
