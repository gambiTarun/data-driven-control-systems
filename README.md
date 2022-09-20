# Data Driven Control Systems

Design of data-driven controller and its application

## Introduction

The real systems have nonlinearity that can be dealt to some extent using a PID control. But systems with high nonlinearly cannot be controlled by a PID controller with fixed parameters. Dynamic parameters allow for better control of systems with strong nonlinearity. In this project, a technique for self-tuning PID control is designed and implemented using recorded data of a system. However, for the controller to work on a nonlinear system the data has be recorded from a nonlinear system.

In the past, many complex control algorithms using the aid of neural networks or genetic algorithms have been proposed for systems with strong nonlinearity. They do come with their drawbacks including high computational cost and large data requirement.

An online data-driven control scheme is designed that reduces the data requirements and thereby the computational costs. The PID parameters are adequately adapted in proportion to the control errors, and the updated parameters are stored in the database. The database is also stripped of any redundant data points in real-time to reduce the computational and memory storage requirements.

Furthermore, an offline data-driven based fictious reference iterative tuning (DD FRIT) control scheme is designed and implemented. In this algorithm, a reference signal is generated from the initial closed-loop data and control parameters, and the optimal PID parameters are calculated using this signal. The major difference from the online method is that new data is not added to the database in real-time.

## Experiment on Tank System

The DD FRIT control method was used on a Tank System for level control. The tank number 2 has a nonlinear relation between its volume and the height, therefore this is a nonlinear system.

The tank system being non-metallic cannot have input of hot water. Therefore, only cold flow was controlled to maintain a level specified by the reference signal. However, the system had a hysteresis which had to be removed before applying the Data Driven FRIT control scheme. For this, a local controller with the actuator was tuned to make the cold flow input and output be linear with each other.

The database is updated for 5 epochs. The following hyper-parameters were set for this example.

After tuning the local PID controller, the main controller was tuned to give barely acceptable results as shown in the figure 14. The input to the system G(s) for the cold flow valve is in percentage of the valve opened. For the control system, the output from the main controller and local controller are saturated from 10% to 40%. This is because the system has the best performance in that range.

We can notice that the input signal u(t) is not strong enough. A stronger input signal can reduce the rise time of the output signal. This is exactly what is achieved by applying the data driven FRIT control scheme. In figure 17, the input signal can be compared to be stronger after getting the PID parameters from the DD FRIT control method.

In this experiment, however, the differential gain KD is forced to be zero. This is because a differential block act as a high pass filter that allow high frequency noise to be amplified. Since the sensor for water level had a lot of noise component, it was better to remove the differential gain from the controller. Hence, KD in figure 16 is zero at all times.


## References

[1] T. Yamamoto, K. Takao and T. Yamada, "Design of a Data-Driven PID Controller," in _IEEE Transactions on Control Systems Technology_, vol. 17, no. 1, pp. 29-39, Jan. 2009. doi: 10.1109/TCST.2008.921808

[2] Wakitani, Shin & Yamamoto, Toru & Gopaluni, Bhushan. (2019). Design and Application of a Database-Driven PID Controller with Data-Driven Updating Algorithm. Industrial & Engineering Chemistry Research. 10.1021/acs.iecr.9b00704.

[3] S. Wakitani and T. Yamamoto, "A learning algorithm for a data-driven controller based on fictitious reference iterative tuning," _2016 International Joint Conference on Neural Networks (IJCNN)_, Vancouver, BC, 2016, pp. 4908-4913. doi: 10.1109/IJCNN.2016.7727845
